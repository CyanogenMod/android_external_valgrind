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
// Author: Timur Iskhodzhanov.
//
// See ts_util.h for mode details.

#include "common_util.h"
#include "thread_sanitizer.h"
#include "ts_stats.h"
#include "ts_lock.h"
#include <stdarg.h>

FLAGS *G_flags = NULL;

#if defined(_MSC_VER)

#pragma comment(lib, "winmm.lib")

# ifdef TS_PIN
#  include "pin.H"
# endif
namespace WINDOWS
{
// This is the way of including winows.h recommended by PIN docs.
#include<Windows.h>
}
int getpid() { return WINDOWS::GetCurrentProcessId(); }
#endif

#if defined(TS_VALGRIND)
size_t TimeInMilliSeconds() {
  return VG_(read_millisecond_timer)();
}
#else
// TODO(kcc): implement this.
size_t TimeInMilliSeconds() {
#ifdef __GNUC__
  return time(0) * 1000;
#else
  return WINDOWS::timeGetTime();
#endif
}
#endif

Stats *G_stats;

bool GetNameAndOffsetOfGlobalObject(uintptr_t addr,
                                    string *name, uintptr_t *offset) {
#ifdef TS_VALGRIND
    const int kBufLen = 1023;
    char buff[kBufLen+1];
    PtrdiffT off;
    if (VG_(get_datasym_and_offset)(addr, reinterpret_cast<Char*>(buff),
                                    kBufLen, &off)) {
      *name = buff;
      *offset = off;
      return true;
    }
    return false;
#else
  return false;
#endif // TS_VALGRIND
}


#ifndef TS_VALGRIND
void GetThreadStack(int tid, uintptr_t *min_addr, uintptr_t *max_addr) {
  *min_addr = 0xfffa;
  *max_addr = 0xfffb;
}
#endif

static int n_errs_found;

void SetNumberOfFoundErrors(int n_errs) {
  n_errs_found = n_errs;
}

int GetNumberOfFoundErrors() {
  return n_errs_found;
}


#if !defined(TS_VALGRIND) && !defined(TS_LLVM)
FILE *G_out = stderr;
#endif

#ifdef TS_LLVM
FILE *G_out;
#endif

static string RemoveUnsupportedFormat(const char *str) {
#ifdef _MSC_VER
  // replace "%'" with "%"
  string res;
  size_t n = strlen(str);
  if (n == 0) {
    return "";
  }
  res.reserve(n);
  res.push_back(str[0]);
  for (size_t i = 1; i < n; i++) {
    if (str[i] == '\'' && *res.rbegin() == '%') continue;
    res.push_back(str[i]);
  }
  return res;
#else
  return str;
#endif
}

void Printf(const char *format, ...) {
#ifdef TS_VALGRIND
  va_list args;
  va_start(args, format);
  VG_(vprintf)(format, args);
  va_end(args);
#else
  va_list args;
  va_start(args, format);
  vfprintf(G_out, RemoveUnsupportedFormat(format).c_str(), args);
  fflush(G_out);
  va_end(args);
#endif
}

// Like Print(), but prepend each line with ==XXXXX==,
// where XXXXX is the pid.
void Report(const char *format, ...) {
  int buff_size = 1024*16;
  char *buff = new char[buff_size];
  CHECK(buff);
  DCHECK(G_flags);

  va_list args;

  while (1) {
    va_start(args, format);
    int ret = vsnprintf(buff, buff_size,
                        RemoveUnsupportedFormat(format).c_str(), args);
    va_end(args);
    if (ret < buff_size) break;
    delete [] buff;
    buff_size *= 2;
    buff = new char[buff_size];
    CHECK(buff);
    // Printf("Resized buff: %d\n", buff_size);
  }

  char pid_buff[100];
  snprintf(pid_buff, sizeof(pid_buff), "==%d== ", getpid());

  string res;
#ifndef TS_LLVM
  int len = strlen(buff);
#else
  int len = __real_strlen(buff);
#endif
  bool last_was_new_line = true;
  for (int i = 0; i < len; i++) {
    if (G_flags->show_pid && last_was_new_line)
      res += pid_buff;
    last_was_new_line = (buff[i] == '\n');
    res += buff[i];
  }

  delete [] buff;

  Printf("%s", res.c_str());
}

long my_strtol(const char *str, char **end, int base) {
#ifdef TS_VALGRIND
  if (base == 16 || (base == 0 && str && str[0] == '0' && str[1] == 'x')) {
    return VG_(strtoll16)((Char*)str, (Char**)end);
  }
  return VG_(strtoll10)((Char*)str, (Char**)end);
#else
  return strtoll(str, end, base);
#endif
}

// Not thread-safe. Need to make it thread-local if we allow
// malloc to be called concurrently.
MallocCostCenterStack g_malloc_stack;

size_t GetVmSizeInMb() {
#ifdef VGO_linux
  const char *path ="/proc/self/statm";  // see 'man proc'
  uintptr_t counter = G_stats->read_proc_self_stats++;
  if (counter >= 1024 && ((counter & (counter - 1)) == 0))
    Report("INFO: reading %s for %ld'th time\n", path, counter);
  int  fd = OpenFileReadOnly(path, false);
  if (fd < 0) return 0;
  char buff[128];
  int n_read = read(fd, buff, sizeof(buff) - 1);
  buff[n_read] = 0;
  close(fd);
  char *end;
  size_t vm_size_in_pages = my_strtol(buff, &end, 10);
  return vm_size_in_pages >> 8;
#else
  return 0;
#endif
}

void OpenFileWriteStringAndClose(const string &file_name, const string &str) {
#ifdef TS_VALGRIND
  SysRes sres = VG_(open)((const Char*)file_name.c_str(),
                          VKI_O_WRONLY|VKI_O_CREAT|VKI_O_TRUNC,
                          VKI_S_IRUSR|VKI_S_IWUSR);
  if (sr_isError(sres)) {
    Report("WARNING: can not open file %s\n", file_name.c_str());
    exit(1);
  }
  int fd = sr_Res(sres);
  write(fd, str.c_str(), str.size());
  close(fd);
#else
  CHECK(0);
#endif
}

//--------- Sockets ------------------ {{{1
#if defined(TS_PIN) && defined(__GNUC__)
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
FILE *OpenSocketForWriting(const string &host_and_port) {
  size_t col = host_and_port.find(":");
  if (col == string::npos) return NULL;
  string host = host_and_port.substr(0, col);
  string port_str = host_and_port.substr(col + 1);
  int sockfd;
  struct sockaddr_in serv_addr;
  struct hostent *server;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) return NULL;
  server = gethostbyname(host.c_str());
  if (server == 0) return NULL;
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  memcpy((char *)&serv_addr.sin_addr.s_addr,
         (char *)server->h_addr,
         server->h_length);
  serv_addr.sin_port = htons(atoi(port_str.c_str()));
  if (connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0)
    return NULL;
  return fdopen(sockfd, "w");
}
#else
FILE *OpenSocketForWriting(const string &host_and_port) {
  return NULL;  // unimplemented.
}
#endif
//--------- TSLock ------------------ {{{1
#ifdef _MSC_VER
//# define TS_LOCK_PIPE
# define TS_LOCK_PIN
#else
# define TS_LOCK_FUTEX
#endif

#if defined(TS_LOCK_PIPE) && defined(TS_PIN)
#ifdef __GNUC__
#include <unistd.h>
// Lock based on pipe's send/receive. The idea (but not the code) 
// is shamelessly stolen from valgrind's /coregrind/m_scheduler/sema.c
struct TSLock::Rep {
  bool held;
  char pipe_char;
  int pipe_fd[2];

  void Write() {
    char buf[2];
    buf[0] = pipe_char;
    buf[1] = 0;
    int res = write(pipe_fd[1], buf, 1);
    CHECK(res == 1);
  }
  bool Read() {
    char buf[2];
    buf[0] = 0;
    buf[1] = 0;
    int res = read(pipe_fd[0], buf, 1);
    if (res != 1)
      return false;
    //Printf("rep::Read: %c\n", buf[0]);

    pipe_char++;
    if (pipe_char == 'Z' + 1) pipe_char = 'A';
    return true;
  }
  void Open() {
    CHECK(0 == pipe(pipe_fd));
    CHECK(pipe_fd[0] != pipe_fd[1]);
    pipe_char = 'A';
  }
  void Close() {
    close(pipe_fd[0]);
    close(pipe_fd[1]);
  }
};
#elif defined(_MSC_VER)
struct TSLock::Rep {
  bool held;
  char pipe_char;
  WINDOWS::HANDLE pipe_fd[2];
  void Write() {
    char buf[2];
    buf[0] = pipe_char;
    buf[1] = 0;
    WINDOWS::DWORD n_written = 0;
    int res = WINDOWS::WriteFile(pipe_fd[1], buf, 1, &n_written, NULL);
    CHECK(res != 0 && n_written == 1);
  }
  bool Read() {
    char buf[2];
    buf[0] = 0;
    buf[1] = 0;
    WINDOWS::DWORD n_read  = 0;
    int res = WINDOWS::ReadFile(pipe_fd[0], buf, 1, &n_read, NULL);
    if (res == 0 && n_read == 0)
      return false;
    //Printf("rep::Read: %c\n", buf[0]);

    pipe_char++;
    if (pipe_char == 'Z' + 1) pipe_char = 'A';
    return true;
  }
  void Open() {
    CHECK(WINDOWS::CreatePipe(&pipe_fd[0], &pipe_fd[1], NULL, 0));
    CHECK(pipe_fd[0] != pipe_fd[1]);
    pipe_char = 'A';
  }
  void Close() {
    WINDOWS::CloseHandle(pipe_fd[0]);
    WINDOWS::CloseHandle(pipe_fd[1]);
  }
};
#endif

TSLock::TSLock() {
  rep_ = new Rep;
  rep_->held = false;
  rep_->Open();
  rep_->Write();
}
TSLock::~TSLock() {
  rep_->Close();
}
void TSLock::Lock() {
  while(rep_->Read() == false)
    ;
  rep_->held = true;
}
void TSLock::Unlock() {
  rep_->held = false;
  rep_->Write();
}
void TSLock::AssertHeld() {
  DCHECK(rep_->held);
}
#endif  // __GNUC__ & TS_LOCK_PIPE

#if defined(TS_LOCK_PIN) && defined(TS_PIN)
#include "pin.H"
struct TSLock::Rep {
  PIN_LOCK lock;
  bool held;
};

TSLock::TSLock() {
  rep_ = new Rep();
  rep_->held = false;
  InitLock(&rep_->lock);
}
TSLock::~TSLock() {
  delete rep_;
}
void TSLock::Lock() {
  GetLock(&rep_->lock, __LINE__);
  rep_->held = true;
}
void TSLock::Unlock() {
  rep_->held = false;
  ReleaseLock(&rep_->lock);
}
void TSLock::AssertHeld() {
  DCHECK(rep_->held);
}
#endif  // TS_LOCK_PIN

#if defined(TS_WRAP_PTHREAD_LOCK)
#include "tsan_rtl_wrap.h"

struct TSLock::Rep {
  pthread_mutex_t lock;
  bool held;
};
TSLock::TSLock() {
  rep_ = new Rep();
  rep_->held = false;
  __real_pthread_mutex_init(&rep_->lock, NULL);
}
TSLock::~TSLock() {
  __real_pthread_mutex_destroy(&rep_->lock);
  delete rep_;
}
void TSLock::Lock() {
  __real_pthread_mutex_lock(&rep_->lock);
  rep_->held = true;
}
void TSLock::Unlock() {
  rep_->held = false;
  __real_pthread_mutex_unlock(&rep_->lock);
}
void TSLock::AssertHeld() {
  DCHECK(rep_->held);
}
#endif  // TS_LLVM

#if defined(TS_LOCK_FUTEX) && defined(__GNUC__) && \
 (defined (TS_PIN) || defined (TS_LLVM))
#include <linux/futex.h>
#include <sys/time.h>
#include <syscall.h>

// Simple futex-based lock.
// The idea is taken from "Futexes Are Tricky" by Ulrich Drepper

TSLock::TSLock() {
  rep_ = 0;
  ANNOTATE_BENIGN_RACE(&rep_, "Benign race on TSLock::rep_");
  ANNOTATE_RWLOCK_CREATE(this);
}
TSLock::~TSLock() {
  ANNOTATE_RWLOCK_DESTROY(this);
  DCHECK(rep_ == 0);
}
void TSLock::Lock() {
  int *p = (int*)&rep_;
  const int kSpinCount = 100;
  DCHECK(kSpinCount > 0);
  int c;
  for (int i = 0; i < kSpinCount; i++) {
    c = __sync_val_compare_and_swap(p, 0, 1);
    if (c == 0) break;
  }
  if (c == 0) {
    // The mutex was unlocked. Now it's ours. Done.
    ANNOTATE_RWLOCK_ACQUIRED(this, /*is_w*/true);
    return;
  }
  DCHECK(c == 1 || c == 2);
  // We are going to block on this lock. Make sure others know that.
  if (c != 2) {
    c = __sync_lock_test_and_set(p, 2);
  }
  // Block.
  int n_waits = 0;
  while (c != 0) {
    syscall(SYS_futex, p, FUTEX_WAIT, 2, 0, 0, 0);
    n_waits++;
    c = __sync_lock_test_and_set(p, 2);
  }
  ANNOTATE_RWLOCK_ACQUIRED(this, /*is_w*/true);
  G_stats->futex_wait += n_waits;
}
void TSLock::Unlock() {
  ANNOTATE_RWLOCK_RELEASED(this, /*is_w*/true);
  int *p = (int*)&rep_;
  DCHECK(*p == 1 || *p == 2);
  int c = __sync_sub_and_fetch(p, 1);
  DCHECK(c == 0 || c == 1);
  if (c == 1) {
    *p = 0;
    syscall(SYS_futex, p, FUTEX_WAKE, 1, 0, 0, 0);
  }
}
void TSLock::AssertHeld() {
  DCHECK(rep_);
}
#endif


//--------------- Atomics ----------------- {{{1
#if defined (_MSC_VER) && TS_SERIALIZED == 0
uintptr_t AtomicExchange(uintptr_t *ptr, uintptr_t new_value) {
  return _InterlockedExchange((volatile WINDOWS::LONG*)ptr, new_value);
}

void ReleaseStore(uintptr_t *ptr, uintptr_t value) {
  *(volatile uintptr_t*)ptr = value;
  // TODO(kcc): anything to add here?
}

int32_t NoBarrier_AtomicIncrement(int32_t* ptr) {
  return _InterlockedIncrement((volatile WINDOWS::LONG *)ptr);
}

int32_t NoBarrier_AtomicDecrement(int32_t* ptr) {
  return _InterlockedDecrement((volatile WINDOWS::LONG *)ptr);
}
#endif  // _MSC_VER && TS_SERIALIZED
//--------------- YIELD ----------------- {{{1
#if defined (_MSC_VER)
void YIELD() {
  WINDOWS::Sleep(0);
}
#elif defined(TS_VALGRIND)
void YIELD() {
}
#elif defined(__GNUC__)
void YIELD() {
  sched_yield();
}
#else
#error "Unknown config"
#endif

// end. {{{1
// vim:shiftwidth=2:softtabstop=2:expandtab:tw=80
