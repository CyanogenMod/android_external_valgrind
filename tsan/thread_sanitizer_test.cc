/* Copyright (c) 2008-2010, Google Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Neither the name of Google Inc. nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// This file is part of ThreadSanitizer, a dynamic data race detector.
// Author: Konstantin Serebryany.

// This file contains tests for various parts of ThreadSanitizer.

#include <gtest/gtest.h>

#include "ts_heap_info.h"
#include "ts_simple_cache.h"

// Testing the HeapMap.
struct TestHeapInfo {
  uintptr_t ptr;
  uintptr_t size;
  int       val;
  TestHeapInfo() : ptr(0), size(0), val(0) { }
  TestHeapInfo(uintptr_t p, uintptr_t s, uintptr_t v) :
    ptr(p), size(s), val(v) { }
};

TEST(ThreadSanitizer, HeapInfoTest) {
  HeapMap<TestHeapInfo> map;
  TestHeapInfo *info;
  EXPECT_EQ(0U, map.size());
  EXPECT_EQ(NULL, map.GetInfo(12345));

  // Insert range [1000, 1000+100) with value 1.
  map.InsertInfo(1000, TestHeapInfo(1000, 100, 1));
  EXPECT_EQ(1U, map.size());
  info = map.GetInfo(1000);
  EXPECT_TRUE(info);
  EXPECT_EQ(1000U, info->ptr);
  EXPECT_EQ(100U, info->size);
  EXPECT_EQ(1, info->val);

  EXPECT_TRUE(map.GetInfo(1000));
  EXPECT_EQ(1, info->val);
  EXPECT_TRUE(map.GetInfo(1050));
  EXPECT_EQ(1, info->val);
  EXPECT_TRUE(map.GetInfo(1099));
  EXPECT_EQ(1, info->val);
  EXPECT_FALSE(map.GetInfo(1100));
  EXPECT_FALSE(map.GetInfo(2000));

  EXPECT_EQ(NULL, map.GetInfo(2000));
  EXPECT_EQ(NULL, map.GetInfo(3000));

  // Insert range [2000, 2000+200) with value 2.
  map.InsertInfo(2000, TestHeapInfo(2000, 200, 2));
  EXPECT_EQ(2U, map.size());

  info = map.GetInfo(1000);
  EXPECT_TRUE(info);
  EXPECT_EQ(1, info->val);

  info = map.GetInfo(2000);
  EXPECT_TRUE(info);
  EXPECT_EQ(2, info->val);

  info = map.GetInfo(1000);
  EXPECT_TRUE(info);
  EXPECT_EQ(1, info->val);
  EXPECT_TRUE((info = map.GetInfo(1050)));
  EXPECT_EQ(1, info->val);
  EXPECT_TRUE((info = map.GetInfo(1099)));
  EXPECT_EQ(1, info->val);
  EXPECT_FALSE(map.GetInfo(1100));

  EXPECT_TRUE((info = map.GetInfo(2000)));
  EXPECT_EQ(2, info->val);
  EXPECT_TRUE((info = map.GetInfo(2199)));
  EXPECT_EQ(2, info->val);

  EXPECT_FALSE(map.GetInfo(2200));
  EXPECT_FALSE(map.GetInfo(3000));

  // Insert range [3000, 3000+300) with value 3.
  map.InsertInfo(3000, TestHeapInfo(3000, 300, 3));
  EXPECT_EQ(3U, map.size());

  EXPECT_TRUE((info = map.GetInfo(1000)));
  EXPECT_EQ(1, info->val);

  EXPECT_TRUE((info = map.GetInfo(2000)));
  EXPECT_EQ(2, info->val);

  EXPECT_TRUE((info = map.GetInfo(3000)));
  EXPECT_EQ(3, info->val);

  EXPECT_TRUE((info = map.GetInfo(1050)));
  EXPECT_EQ(1, info->val);

  EXPECT_TRUE((info = map.GetInfo(2100)));
  EXPECT_EQ(2, info->val);

  EXPECT_TRUE((info = map.GetInfo(3200)));
  EXPECT_EQ(3, info->val);

  // Remove range [2000,2000+200)
  map.EraseInfo(2000);
  EXPECT_EQ(2U, map.size());

  EXPECT_TRUE((info = map.GetInfo(1050)));
  EXPECT_EQ(1, info->val);

  EXPECT_FALSE(map.GetInfo(2100));

  EXPECT_TRUE((info = map.GetInfo(3200)));
  EXPECT_EQ(3, info->val);

}

TEST(ThreadSanitizer, PtrToBoolCacheTest) {
  PtrToBoolCache<256> c;
  bool val = false;
  EXPECT_FALSE(c.Lookup(123, &val));

  c.Insert(0, false);
  c.Insert(1, true);
  c.Insert(2, false);
  c.Insert(3, true);

  EXPECT_TRUE(c.Lookup(0, &val));
  EXPECT_EQ(false, val);
  EXPECT_TRUE(c.Lookup(1, &val));
  EXPECT_EQ(true, val);
  EXPECT_TRUE(c.Lookup(2, &val));
  EXPECT_EQ(false, val);
  EXPECT_TRUE(c.Lookup(3, &val));
  EXPECT_EQ(true, val);

  EXPECT_FALSE(c.Lookup(256, &val));
  EXPECT_FALSE(c.Lookup(257, &val));
  EXPECT_FALSE(c.Lookup(258, &val));
  EXPECT_FALSE(c.Lookup(259, &val));

  c.Insert(0, true);
  c.Insert(1, false);

  EXPECT_TRUE(c.Lookup(0, &val));
  EXPECT_EQ(true, val);
  EXPECT_TRUE(c.Lookup(1, &val));
  EXPECT_EQ(false, val);
  EXPECT_TRUE(c.Lookup(2, &val));
  EXPECT_EQ(false, val);
  EXPECT_TRUE(c.Lookup(3, &val));
  EXPECT_EQ(true, val);

  c.Insert(256, false);
  c.Insert(257, false);
  EXPECT_FALSE(c.Lookup(0, &val));
  EXPECT_FALSE(c.Lookup(1, &val));
  EXPECT_TRUE(c.Lookup(2, &val));
  EXPECT_EQ(false, val);
  EXPECT_TRUE(c.Lookup(3, &val));
  EXPECT_EQ(true, val);
  EXPECT_TRUE(c.Lookup(256, &val));
  EXPECT_EQ(false, val);
  EXPECT_TRUE(c.Lookup(257, &val));
  EXPECT_EQ(false, val);
}

TEST(ThreadSanitizer, IntPairToBoolCacheTest) {
  IntPairToBoolCache<257> c;
  bool val = false;
  map<pair<int,int>, bool> m;

  for (int i = 0; i < 1000000; i++) {
    int a = (rand() % 1024) + 1;
    int b = (rand() % 1024) + 1;

    if (c.Lookup(a, b, &val)) {
      EXPECT_EQ(1U, m.count(make_pair(a,b)));
      EXPECT_EQ(val, m[make_pair(a,b)]);
    }

    val = (rand() % 2) == 1;
    c.Insert(a, b, val);
    m[make_pair(a,b)] = val;
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
