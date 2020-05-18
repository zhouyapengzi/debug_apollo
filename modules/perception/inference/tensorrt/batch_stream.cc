/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/inference/tensorrt/batch_stream.h"

#include <algorithm>

#include "absl/strings/str_cat.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace inference {

BatchStream::BatchStream(int batchSize, int maxBatches, std::string dataPath)
    : mBatchSize(batchSize), mMaxBatches(maxBatches), mPath(dataPath) {
    AINFO<<"(DMCZP) EnteringMethod: BatchStream::BatchStream";

  FILE *file = fopen((mPath + "Batch0").c_str(), "rb");
  if (file != nullptr) {
    int d[4];
    int fs = static_cast<int>(fread(d, sizeof(int), 4, file));
    CHECK_EQ(fs, 4);
    mDims = nvinfer1::DimsNCHW{d[0], d[1], d[2], d[3]};
    fclose(file);
    mImageSize = mDims.c() * mDims.h() * mDims.w();
    mBatch.resize(mBatchSize * mImageSize, 0);
    mFileBatch.resize(mDims.n() * mImageSize, 0);
    reset(0);
  }

   AINFO<<"(DMCZP) LeaveMethod: BatchStream::BatchStream";
 }

BatchStream::BatchStream() : mPath("") {
    AINFO<<"(DMCZP) EnteringMethod: BatchStream::BatchStream";

   AINFO<<"(DMCZP) LeaveMethod: BatchStream::BatchStream";
 }

void BatchStream::reset(int firstBatch) {
    AINFO<<"(DMCZP) EnteringMethod: BatchStream::reset";

  if (mPath != "") {
    mBatchCount = 0;
    mFileCount = 0;
    mFileBatchPos = mDims.n();
    skip(firstBatch);
  }

   AINFO<<"(DMCZP) LeaveMethod: BatchStream::reset";
 }

bool BatchStream::next() {
    AINFO<<"(DMCZP) EnteringMethod: BatchStream::next";

  if (mBatchCount == mMaxBatches) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: BatchStream::next";
  return false;
  }

  for (int csize = 1, batchPos = 0; batchPos < mBatchSize;
       batchPos += csize, mFileBatchPos += csize) {
    CHECK_GT(mFileBatchPos, 0);
    CHECK_LE(mFileBatchPos, mDims.n());
    // mMaxBatches > number of batches in the files
    if (mFileBatchPos == mDims.n() && !update()) {
      
  AINFO<<"(DMCZP) (return) LeaveMethod: BatchStream::next";
  return false;
    }

    // copy the smaller of: elements left to fulfill the request,
    // or elements left in the file buffer.
    csize = std::min(mBatchSize - batchPos, mDims.n() - mFileBatchPos);
    std::copy_n(getFileBatch() + mFileBatchPos * mImageSize, csize * mImageSize,
                getBatch() + batchPos * mImageSize);
  }
  mBatchCount++;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: BatchStream::next";
  return true;

   AINFO<<"(DMCZP) LeaveMethod: BatchStream::next";
 }

void BatchStream::skip(int skipCount) {
    AINFO<<"(DMCZP) EnteringMethod: BatchStream::skip";

  if (mBatchSize >= mDims.n() && mBatchSize % mDims.n() == 0 &&
      mFileBatchPos == mDims.n()) {
    mFileCount += skipCount * mBatchSize / mDims.n();
    
  AINFO<<"(DMCZP) (return) LeaveMethod: BatchStream::skip";
  return;
  }

  int x = mBatchCount;
  for (int i = 0; i < skipCount; i++) {
    next();
  }
  mBatchCount = x;

   AINFO<<"(DMCZP) LeaveMethod: BatchStream::skip";
 }

bool BatchStream::update() {
    AINFO<<"(DMCZP) EnteringMethod: BatchStream::update";

  std::string inputFileName = absl::StrCat(mPath, "Batch", mFileCount++);
  FILE *file = fopen(inputFileName.c_str(), "rb");
  if (file == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: BatchStream::update";
  return false;
  }

  int d[4];
  int fs = static_cast<int>(fread(d, sizeof(int), 4, file));
  CHECK_EQ(fs, 4);
  CHECK(mDims.n() == d[0] && mDims.c() == d[1] && mDims.h() == d[2] &&
        mDims.w() == d[3]);

  size_t readInputCount =
      fread(getFileBatch(), sizeof(float), mDims.n() * mImageSize, file);
  CHECK_EQ(readInputCount, size_t(mDims.n() * mImageSize));
  fclose(file);
  mFileBatchPos = 0;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: BatchStream::update";
  return true;

   AINFO<<"(DMCZP) LeaveMethod: BatchStream::update";
 }

}  // namespace inference
}  // namespace perception
}  // namespace apollo
