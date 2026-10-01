#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

#include "../Memory/Memory.h"

struct BmpHeader;

// Helper functions
uint8_t quantize(int gray, int x, int y);
uint8_t quantizeSimple(int gray);
uint8_t quantize1bit(int gray, int x, int y);
int adjustPixel(int gray);

struct GrayPlanePixel {
  bool write;
  bool black;
};

inline uint8_t applyJpegQualityTone(uint8_t gray, uint8_t leftGray, uint8_t rightGray, int x, int y) {
  constexpr int solidBlackMax = 12;
  constexpr int solidWhiteMin = 218;
  constexpr int shadowContrastPercent = 122;
  constexpr int contrastPercent = 162;
  constexpr int sharpenThreshold = 3;
  constexpr int sharpenPercent = 105;
  constexpr int sharpenMax = 38;
  constexpr int microDither = 8;

  if (gray <= solidBlackMax) return 0;
  if (gray >= solidWhiteMin) return 255;

  const int detail = static_cast<int>(gray) - (static_cast<int>(leftGray) + rightGray) / 2;
  int sharpenedGray = gray;
  const int absDetail = detail < 0 ? -detail : detail;
  if (absDetail > sharpenThreshold) {
    int boost = (detail * sharpenPercent) / 100;
    if (boost < -sharpenMax) boost = -sharpenMax;
    if (boost > sharpenMax) boost = sharpenMax;
    sharpenedGray += boost;
    if (sharpenedGray < 0) sharpenedGray = 0;
    if (sharpenedGray > 255) sharpenedGray = 255;
  }

  int tone;
  if (sharpenedGray < 128) {
    tone = ((sharpenedGray - 64) * shadowContrastPercent) / 100 + 64;
  } else {
    tone = ((sharpenedGray - 128) * contrastPercent) / 100 + 128;
  }
  if (tone <= 8) return 0;
  if (tone >= 238) return 255;

  if (gray > solidBlackMax + 10 && gray < solidWhiteMin - 10) {
    const int latticeA = ((x * 13 + y * 7 + ((x ^ y) * 3)) & 15) - 8;
    const int latticeB = (((x + y * 3) * 5) & 7) - 4;
    tone += ((latticeA + latticeB) * microDither) / 12;
  }

  if (tone < 0) tone = 0;
  if (tone > 255) tone = 255;
  return static_cast<uint8_t>(tone);
}

// Levels: black, dark, light, white. drawPixel(true) clears a framebuffer bit.
constexpr GrayPlanePixel grayPlanePixel(uint8_t level, bool msb, bool absolute) {
  if (absolute) return {true, !(level == 3 || level == (msb ? 2 : 1))};
  return {msb ? (level == 1 || level == 2) : level == 1, false};
}

enum class BmpRowOrder { BottomUp, TopDown };

// Populates a 1-bit BMP header in the provided memory.
void createBmpHeader(BmpHeader* bmpHeader, int width, int height, BmpRowOrder rowOrder);

// 1-bit Atkinson dithering - better quality than noise dithering for thumbnails
// Error distribution pattern (same as 2-bit but quantizes to 2 levels):
//     X  1/8 1/8
// 1/8 1/8 1/8
//     1/8
class Atkinson1BitDitherer {
 public:
  explicit Atkinson1BitDitherer(int width) {
    if (width <= 0) return;
    const size_t candidateRowSize = static_cast<size_t>(width) + 4;
    if (candidateRowSize > SIZE_MAX / (3 * sizeof(int16_t))) return;
    rowSize = candidateRowSize;
    errorRows = makeUniqueNoThrow<int16_t[]>(rowSize * 3);
    if (!errorRows) return;
    errorRow0 = errorRows.get();
    errorRow1 = errorRow0 + rowSize;
    errorRow2 = errorRow1 + rowSize;
  }

  bool isValid() const { return errorRows != nullptr; }

  // EXPLICITLY DELETE THE COPY CONSTRUCTOR
  Atkinson1BitDitherer(const Atkinson1BitDitherer& other) = delete;

  // EXPLICITLY DELETE THE COPY ASSIGNMENT OPERATOR
  Atkinson1BitDitherer& operator=(const Atkinson1BitDitherer& other) = delete;

  uint8_t processPixel(int gray, int x) {
    if (!isValid()) return adjustPixel(gray) < 128 ? 0 : 1;

    // Apply brightness/contrast/gamma adjustments
    gray = adjustPixel(gray);

    // Add accumulated error
    int adjusted = gray + errorRow0[x + 2];
    if (adjusted < 0) adjusted = 0;
    if (adjusted > 255) adjusted = 255;

    // Quantize to 2 levels (1-bit): 0 = black, 1 = white
    uint8_t quantized;
    int quantizedValue;
    if (adjusted < 128) {
      quantized = 0;
      quantizedValue = 0;
    } else {
      quantized = 1;
      quantizedValue = 255;
    }

    // Calculate error (only distribute 6/8 = 75%)
    int error = (adjusted - quantizedValue) >> 3;  // error/8

    // Distribute 1/8 to each of 6 neighbors
    errorRow0[x + 3] += error;  // Right
    errorRow0[x + 4] += error;  // Right+1
    errorRow1[x + 1] += error;  // Bottom-left
    errorRow1[x + 2] += error;  // Bottom
    errorRow1[x + 3] += error;  // Bottom-right
    errorRow2[x + 2] += error;  // Two rows down

    return quantized;
  }

  void nextRow() {
    if (!isValid()) return;
    int16_t* temp = errorRow0;
    errorRow0 = errorRow1;
    errorRow1 = errorRow2;
    errorRow2 = temp;
    memset(errorRow2, 0, rowSize * sizeof(int16_t));
  }

  void reset() {
    if (!isValid()) return;
    memset(errorRow0, 0, rowSize * sizeof(int16_t));
    memset(errorRow1, 0, rowSize * sizeof(int16_t));
    memset(errorRow2, 0, rowSize * sizeof(int16_t));
  }

 private:
  size_t rowSize{0};
  std::unique_ptr<int16_t[]> errorRows;
  int16_t* errorRow0 = nullptr;
  int16_t* errorRow1 = nullptr;
  int16_t* errorRow2 = nullptr;
};

// Atkinson dithering - distributes only 6/8 (75%) of error for cleaner results
// Error distribution pattern:
//     X  1/8 1/8
// 1/8 1/8 1/8
//     1/8
// Less error buildup = fewer artifacts than Floyd-Steinberg
class AtkinsonDitherer {
 public:
  explicit AtkinsonDitherer(int width, bool imageLevels = false) : imageLevels(imageLevels) {
    if (width <= 0) return;
    const size_t candidateRowSize = static_cast<size_t>(width) + 4;
    if (candidateRowSize > SIZE_MAX / (3 * sizeof(int16_t))) return;
    rowSize = candidateRowSize;
    errorRows = makeUniqueNoThrow<int16_t[]>(rowSize * 3);
    if (!errorRows) return;
    errorRow0 = errorRows.get();
    errorRow1 = errorRow0 + rowSize;
    errorRow2 = errorRow1 + rowSize;
  }
  // **1. EXPLICITLY DELETE THE COPY CONSTRUCTOR**
  AtkinsonDitherer(const AtkinsonDitherer& other) = delete;

  // **2. EXPLICITLY DELETE THE COPY ASSIGNMENT OPERATOR**
  AtkinsonDitherer& operator=(const AtkinsonDitherer& other) = delete;

  bool isValid() const { return errorRows != nullptr; }

  uint8_t processPixel(int gray, int x) {
    // Add accumulated error
    int adjusted = gray + (isValid() ? errorRow0[x + 2] : 0);
    if (adjusted < 0) adjusted = 0;
    if (adjusted > 255) adjusted = 255;

    // Quantize to 4 levels
    uint8_t quantized;
    int quantizedValue;
    if (imageLevels) {  // evenly spaced image tones
      if (adjusted < 43) {
        quantized = 0;
        quantizedValue = 0;
      } else if (adjusted < 128) {
        quantized = 1;
        quantizedValue = 85;
      } else if (adjusted < 213) {
        quantized = 2;
        quantizedValue = 170;
      } else {
        quantized = 3;
        quantizedValue = 255;
      }
    } else {  // fine-tuned to X4 eink display
      if (adjusted < 30) {
        quantized = 0;
        quantizedValue = 15;
      } else if (adjusted < 50) {
        quantized = 1;
        quantizedValue = 30;
      } else if (adjusted < 140) {
        quantized = 2;
        quantizedValue = 80;
      } else {
        quantized = 3;
        quantizedValue = 210;
      }
    }

    if (!isValid()) return quantized;

    // Calculate error (only distribute 6/8 = 75%)
    int error = (adjusted - quantizedValue) >> 3;  // error/8

    // Distribute 1/8 to each of 6 neighbors
    errorRow0[x + 3] += error;  // Right
    errorRow0[x + 4] += error;  // Right+1
    errorRow1[x + 1] += error;  // Bottom-left
    errorRow1[x + 2] += error;  // Bottom
    errorRow1[x + 3] += error;  // Bottom-right
    errorRow2[x + 2] += error;  // Two rows down

    return quantized;
  }

  void nextRow() {
    if (!isValid()) return;
    int16_t* temp = errorRow0;
    errorRow0 = errorRow1;
    errorRow1 = errorRow2;
    errorRow2 = temp;
    memset(errorRow2, 0, rowSize * sizeof(int16_t));
  }

  void reset() {
    if (!isValid()) return;
    memset(errorRow0, 0, rowSize * sizeof(int16_t));
    memset(errorRow1, 0, rowSize * sizeof(int16_t));
    memset(errorRow2, 0, rowSize * sizeof(int16_t));
  }

 private:
  const bool imageLevels;
  size_t rowSize{0};
  std::unique_ptr<int16_t[]> errorRows;
  int16_t* errorRow0 = nullptr;
  int16_t* errorRow1 = nullptr;
  int16_t* errorRow2 = nullptr;
};

// Floyd-Steinberg error diffusion dithering with serpentine scanning
// Serpentine scanning alternates direction each row to reduce "worm" artifacts
// Error distribution pattern (left-to-right):
//       X   7/16
// 3/16 5/16 1/16
// Error distribution pattern (right-to-left, mirrored):
// 1/16 5/16 3/16
//      7/16  X
class FloydSteinbergDitherer {
 public:
  explicit FloydSteinbergDitherer(int width, bool imageLevels = false) : imageLevels(imageLevels), rowCount(0) {
    if (width <= 0) return;
    const size_t candidateRowSize = static_cast<size_t>(width) + 2;
    if (candidateRowSize > SIZE_MAX / (2 * sizeof(int16_t))) return;
    rowSize = candidateRowSize;
    errorRows = makeUniqueNoThrow<int16_t[]>(rowSize * 2);
    if (!errorRows) return;
    errorCurRow = errorRows.get();
    errorNextRow = errorCurRow + rowSize;
  }

  bool isValid() const { return errorRows != nullptr; }

  // **1. EXPLICITLY DELETE THE COPY CONSTRUCTOR**
  FloydSteinbergDitherer(const FloydSteinbergDitherer& other) = delete;

  // **2. EXPLICITLY DELETE THE COPY ASSIGNMENT OPERATOR**
  FloydSteinbergDitherer& operator=(const FloydSteinbergDitherer& other) = delete;

  // Process a single pixel and return quantized 2-bit value
  // x is the logical x position (0 to width-1), direction handled internally
  uint8_t processPixel(int gray, int x) {
    // Add accumulated error to this pixel
    int adjusted = gray + (isValid() ? errorCurRow[x + 1] : 0);

    // Clamp to valid range
    if (adjusted < 0) adjusted = 0;
    if (adjusted > 255) adjusted = 255;

    // Quantize to 4 levels (0, 85, 170, 255)
    uint8_t quantized;
    int quantizedValue;
    if (imageLevels) {  // evenly spaced image tones
      if (adjusted < 43) {
        quantized = 0;
        quantizedValue = 0;
      } else if (adjusted < 128) {
        quantized = 1;
        quantizedValue = 85;
      } else if (adjusted < 213) {
        quantized = 2;
        quantizedValue = 170;
      } else {
        quantized = 3;
        quantizedValue = 255;
      }
    } else {  // fine-tuned to X4 eink display
      if (adjusted < 30) {
        quantized = 0;
        quantizedValue = 15;
      } else if (adjusted < 50) {
        quantized = 1;
        quantizedValue = 30;
      } else if (adjusted < 140) {
        quantized = 2;
        quantizedValue = 80;
      } else {
        quantized = 3;
        quantizedValue = 210;
      }
    }

    if (!isValid()) return quantized;

    // Calculate error
    int error = adjusted - quantizedValue;

    // Distribute error to neighbors (serpentine: direction-aware)
    if (!isReverseRow()) {
      // Left to right: standard distribution
      // Right: 7/16
      errorCurRow[x + 2] += (error * 7) >> 4;
      // Bottom-left: 3/16
      errorNextRow[x] += (error * 3) >> 4;
      // Bottom: 5/16
      errorNextRow[x + 1] += (error * 5) >> 4;
      // Bottom-right: 1/16
      errorNextRow[x + 2] += (error) >> 4;
    } else {
      // Right to left: mirrored distribution
      // Left: 7/16
      errorCurRow[x] += (error * 7) >> 4;
      // Bottom-right: 3/16
      errorNextRow[x + 2] += (error * 3) >> 4;
      // Bottom: 5/16
      errorNextRow[x + 1] += (error * 5) >> 4;
      // Bottom-left: 1/16
      errorNextRow[x] += (error) >> 4;
    }

    return quantized;
  }

  // Call at the end of each row to swap buffers
  void nextRow() {
    if (!isValid()) return;
    // Swap buffers
    int16_t* temp = errorCurRow;
    errorCurRow = errorNextRow;
    errorNextRow = temp;
    // Clear the next row buffer
    memset(errorNextRow, 0, rowSize * sizeof(int16_t));
    rowCount++;
  }

  // Check if current row should be processed in reverse
  bool isReverseRow() const { return (rowCount & 1) != 0; }

  // Reset for a new image or MCU block
  void reset() {
    if (!isValid()) return;
    memset(errorCurRow, 0, rowSize * sizeof(int16_t));
    memset(errorNextRow, 0, rowSize * sizeof(int16_t));
    rowCount = 0;
  }

 private:
  const bool imageLevels;
  int rowCount;
  size_t rowSize{0};
  std::unique_ptr<int16_t[]> errorRows;
  int16_t* errorCurRow = nullptr;
  int16_t* errorNextRow = nullptr;
};

class QualityFourToneDitherer {
 public:
  explicit QualityFourToneDitherer(int width) {
    if (width <= 0) return;
    rowSize = static_cast<size_t>(width) + 4;
    if (rowSize > SIZE_MAX / (2 * sizeof(int16_t))) return;
    width_ = width;
    errorRows = makeUniqueNoThrow<int16_t[]>(rowSize * 2);
    if (errorRows) {
      currentRow = errorRows.get();
      nextRowBuffer = currentRow + rowSize;
    }
  }

  QualityFourToneDitherer(const QualityFourToneDitherer&) = delete;
  QualityFourToneDitherer& operator=(const QualityFourToneDitherer&) = delete;

  bool isValid() const { return errorRows != nullptr; }

  static uint8_t quantize(int gray) {
    if (gray < 20) return 0;
    if (gray < 158) return 1;
    if (gray < 248) return 2;
    return 3;
  }

  uint8_t processPixel(int gray, int x) {
    if (x < 0 || x >= width_ || !isValid()) return quantize(gray);

    int adjusted = gray + currentRow[x + 2];
    if (adjusted < 0) adjusted = 0;
    if (adjusted > 255) adjusted = 255;

    const uint8_t level = quantize(adjusted);
    const int error = adjusted - static_cast<int>(level) * 85;
    if (error == 0) return level;

    if (x + 1 < width_) currentRow[x + 3] += static_cast<int16_t>((error * 7) / 16);
    if (x > 0) nextRowBuffer[x + 1] += static_cast<int16_t>((error * 3) / 16);
    nextRowBuffer[x + 2] += static_cast<int16_t>((error * 5) / 16);
    if (x + 1 < width_) nextRowBuffer[x + 3] += static_cast<int16_t>(error / 16);
    return level;
  }

  void nextRow() {
    if (!isValid()) return;
    int16_t* previous = currentRow;
    currentRow = nextRowBuffer;
    nextRowBuffer = previous;
    memset(nextRowBuffer, 0, rowSize * sizeof(int16_t));
  }

 private:
  int width_ = 0;
  size_t rowSize = 0;
  std::unique_ptr<int16_t[]> errorRows;
  int16_t* currentRow = nullptr;
  int16_t* nextRowBuffer = nullptr;
};
