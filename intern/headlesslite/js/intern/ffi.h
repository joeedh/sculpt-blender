#ifndef _FFI_H
#define _FFI_H

//#include "v8.h"
//using namespace v8;

/*int ArgI(Local<Value>[] args, int *variable, int *state) {
  *state++;
  *variable = some_value;
  if error happened, return 1 else 0
}
//*/
typedef _MYVEC3 float[3];

#define PTYPE void*
#define ITYPE int
#define VEC3TYPE _MYVEC3

#define FBASE(name, narg, rtype)\
static void WRAP_##name(const v8::FunctionCallbackInfo<v8::Value>& args) {\
  Isolate* isolate = args.GetIsolate();\
  EscapableHandleScope scope(isolate);\
  int state = 0;\
  rtype ret;

#define FEND(rtype)\
  cb()\
  _set_##rtype(ret);\
}

//stage a
#define FSA1(t1)\
  t1##TYPE a = Arg##t1(args, &state);
#define FSA2(t1, t2) FSA1(t1);\
  t2##TYPE b = Arg##t1(args, &state);
#define FSA3(t1, t2, t3) FSA2(t1, t2);\
  t3##TYPE c = Arg##t1(args, &state);
#define FSA4(t1, t2, t3, t4) FSA3(t1, t2, t3);\
  t4##TYPE d = Arg##t1(args, &state);

#define FSB1(name) ret = name(a);
#define FSB2(name) ret = name(a, b);
#define FSB3(name) ret = name(a, b, c);
#define FSB4(name) ret = name(a, b, c, d);

#define WRAP(narg, name, rtype, ...)\
  FBASE(name, narg, rtype)\
  FSA##narg(__VA_ARGS__)\
  FSB##narg(name)

#endif /* _FFI_H */
