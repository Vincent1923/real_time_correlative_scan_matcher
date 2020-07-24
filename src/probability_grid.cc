#include "probability_grid.h"

#include <limits>

float ProbabilityGrid::GetProbability(const Eigen::Array2i& cell_index) const {
  // 通过父类的接口判定一下输入的单元索引是否在子图的范围内，如果不在就直接返回一个最小的概率。
  if (!limits().Contains(cell_index)) return 0.1f;
  /**
   * 1. 获取栅格对应的占用概率并返回。
   *    调用了两个在文件 "probability_values.h" 中定义的函数。
   * 2. 函数 ValueToCorrespondenceCost() 是将 uint16 类型的数据转换为 float 类型，
   *    并将输入从区间 [1,32767] 映射到 [kMinCorrespondenceCost,kMaxCorrespondenceCost]。
   * 3. 函数 CorrespondenceCostToProbability() 是对输入概率值取反，由于栅格单元存储的是空闲(free)的概率，
   *    所以这里实际返回的是栅格单元的占用(occupancy)概率。
   */
  return 0.1;
}
