#ifndef _JSMAIN_H
#define _JSMAIN_H

#include "v8.h"
#include <vector>

using v8::Primitive;
using v8::Value;
using v8::Isolate;
using v8::Global;
using v8::Persistent;
using v8::Array;
using v8::Local;
using v8::Script;
using v8::String;
using v8::Object;
using v8::ObjectTemplate;
using v8::MaybeLocal;
using v8::HandleScope;
using v8::Context;
using v8::EscapableHandleScope;
using v8::External;
using v8::Function;
using v8::FunctionTemplate;

namespace headlesslite {

class JSState;

class JSTimer {
public:
  JSTimer(Isolate *isolate, int id, Local<Function> callback, int ms) {
    id = id;
    ms_ = ms;
    cb_.Reset(isolate, callback);
    lastTime_ = 0;
  }

  ~JSTimer() {
    cb_.Reset();
  }

  void update(JSState *state);

  bool dead() {
    return true; //XXX
  }

  int id;

private:
  int ms_;
  Global<Function> cb_;
  double lastTime_;
};

class JSState {
public:
  JSState();
  ~JSState();

  void handleEvents() {
    int i;

    for (i = 0; i < timers_.size(); i++) {
      timers_[i]->update(this);
    }
  }

  int addTimer(Local<Function> callback, int ms) {
    int id = timer_idgen_++;
    JSTimer *timer = new JSTimer(GetIsolate(), id, callback, ms);
    this->timers_.push_back(timer);

    return id;
  }

  int removeTimer(int id) {
    for (int i = 0; i < timers_.size(); i++) {
      if (timers_[i]->id == id) {
        //XXX use proper copy semantics
        if (timers_.size() > 1 && i != timers_.size() - 1) {
          std::swap(timers_[timers_.size() - 1], timers_[i]);
        }

        timers_.pop_back();
        break;
      }
    }
  }

  double timeMS() {
    return platform_->CurrentClockTimeMillis();
  }

  bool haveEvents() {
    return timers_.size() > 0;
  }

  Local<Value> ExecuteScript(Local<String> script, int *error);
  Isolate* GetIsolate() { return isolate_; }
  Local<Context> GetContext() {
    EscapableHandleScope scope(GetIsolate());
    return scope.Escape(context_.Get(GetIsolate()));
  }
private:
  std::vector<JSTimer*> timers_;

  int timer_idgen_ = 0;
  Isolate* isolate_;
  Global<Context> context_;
  Global<Function> process_;
  Global<ObjectTemplate> global_;
  std::unique_ptr<v8::Platform> platform_;
};

}

#endif /* _JSMAIN_H */
