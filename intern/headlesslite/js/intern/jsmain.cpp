#include "v8.h"
#include "libplatform/libplatform.h"

#include <vector>
#include <map>

//#define NO_V8

#define MYDLLINTERN
#include "jsc_api.h"
static FILE *log_file = 0;

#ifndef NO_V8
#include "jsmain.h"

using namespace headlesslite;

using std::map;
using std::pair;
using std::string;

using v8::Context;
using v8::EscapableHandleScope;
using v8::External;
using v8::Integer;
using v8::Function;
using v8::FunctionTemplate;
using v8::Message;
using v8::Global;
using v8::HandleScope;
using v8::Isolate;
using v8::Int32;
using v8::Local;
using v8::MaybeLocal;
using v8::Name;
using v8::NamedPropertyHandlerConfiguration;
using v8::NewStringType;
using v8::Object;
using v8::ObjectTemplate;
using v8::PropertyCallbackInfo;
using v8::Script;
using v8::String;
using v8::TryCatch;
using v8::Value;

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

extern char jscode[];

JSState *state = 0;

void JSTimer::update(JSState *state) {
	if (state->timeMS() - lastTime_ > ms_) {
		Isolate *isolate = state->GetIsolate();
		HandleScope scope(isolate);
		Local<Context> context = state->GetContext();
		Context::Scope context_scope(context);
		EscapableHandleScope handle_scope(isolate);
		TryCatch try_catch(isolate);


		Local<Value> call_args[1] = { v8::Undefined(isolate) };

		Local<Function> cb = cb_.Get(isolate);
		cb->Call(context, context->Global(), 0, call_args);
		lastTime_ = state->timeMS();
	}
}

void Log(char *msg) {
	if (log_file) {
		fprintf(log_file, "%s\n", msg);
		fflush(log_file);
	}
}

static void NativePrintCB(const v8::FunctionCallbackInfo<v8::Value>& args) {
	if (args.Length() < 1) return;

	Isolate* isolate = args.GetIsolate();
	HandleScope scope(isolate);
	Local<Value> arg = args[0];
	String::Utf8Value value(isolate, arg);

	if (log_file) {
		fprintf(log_file, "%s", *value);
		fflush(log_file);
	}
}

static void setIntervalCB(const v8::FunctionCallbackInfo<v8::Value>& args) {

	if (args.Length() < 2) return;
	Isolate* isolate = args.GetIsolate();
	EscapableHandleScope scope(isolate);

	if (!args[0]->IsFunction()) {
		isolate->ThrowException(String::NewFromUtf8(isolate, "Expected a function for first argument", 
			NewStringType::kNormal).ToLocalChecked());
			return;
	}

	if (!args[1]->IsNumber()) {
		isolate->ThrowException(String::NewFromUtf8(isolate, "Expected a number for second argument",
			NewStringType::kNormal).ToLocalChecked());
		return;
	}

	Local<Function> func = Local<Function>::Cast(args[0]);
	Local<Int32> jsms;

	args[1]->ToInt32(state->GetContext()).ToLocal(&jsms);
	int id = state->addTimer(func, jsms->Int32Value(state->GetContext()).ToChecked());
	
	Local<Integer> jid = Integer::New(isolate, (int32_t)id);

	args.GetReturnValue().Set(jid);
}

JSState::JSState() {
	v8::V8::InitializeICUDefaultLocation(".");
	v8::V8::InitializeExternalStartupData(".");
	platform_ = v8::platform::NewDefaultPlatform();
	v8::V8::InitializePlatform(platform_.get());
	v8::V8::Initialize();
	
	Isolate::CreateParams create_params;
	create_params.array_buffer_allocator =
		v8::ArrayBuffer::Allocator::NewDefaultAllocator();
	Isolate* isolate = Isolate::New(create_params);
	Isolate::Scope isolate_scope(isolate);
	HandleScope scope(isolate);
	Local<String> source;

	isolate_ = isolate;

	String::NewFromUtf8(isolate, jscode, NewStringType::kNormal, static_cast<int>(strlen(jscode))).ToLocal(&source);
	// Create a handle scope to hold the temporary references.

	// Create a template for the global object where we set the
	// built-in global functions.
	Local<ObjectTemplate> global = ObjectTemplate::New(GetIsolate());
	global->Set(String::NewFromUtf8(GetIsolate(), "_native_print", NewStringType::kNormal)
		.ToLocalChecked(),
		FunctionTemplate::New(GetIsolate(), NativePrintCB));
	global->Set(String::NewFromUtf8(GetIsolate(), "setInterval", NewStringType::kNormal)
		.ToLocalChecked(),
		FunctionTemplate::New(GetIsolate(), setIntervalCB));

	global_.Reset(GetIsolate(), global);

	//context2_ = new Persistent<Context>(GetIsolate(), Context::New(GetIsolate(), NULL, global));
	//context2_ = new Persistent<Context>(GetIsolate(), ctx);

	context_.Reset(GetIsolate(), Context::New(GetIsolate(), NULL, global));

	// Enter the new context so all the following operations take place
	// within it.
	Context::Scope context_scope(context_.Get(isolate));

	int error = 0;
	
	Local<Value> result = ExecuteScript(source, &error);

	if (error) {
		//handle somehow?
	} else {
		Persistent<Value> *p = new Persistent<Value>(isolate, result);
	}
}

JSState::~JSState() {
	// Dispose the persistent handles.  When no one else has any
	// references to the objects stored in the handles they will be
	// automatically reclaimed.
	context_.Reset();
	process_.Reset();
	//delete context2_;
}

static void format_script_error(TryCatch &try_catch, Isolate *isolate) {
	HandleScope handle_scope(isolate);
	char _buf[512], *smsg;
	int dynamic = 0;
	Local<Message> msg = try_catch.Message();
	Local<Context> context(isolate->GetCurrentContext());

	int line = 0;

	msg->GetLineNumber(context).To(&line);
	Local<String> str = msg->Get();

	int len = str->Utf8Length(isolate);
	//how's this for a margin of error? multiply sizeof(_buf) by 2! :)
	dynamic = len >= sizeof(_buf)*2;

	if (dynamic) {
		smsg = (char*) malloc(len * 2);
	} else {
		smsg = _buf;
		len = sizeof(_buf);
	}

	int written = 0;

	String::Utf8Value error(isolate, try_catch.Exception());

	str->WriteUtf8(isolate, smsg, len, &written);
	sprintf_s(smsg + written, len-written, "\n\nError:%d\n%s\n", line, *error);
	smsg[len - 1] = 0; //paranoia null-termination check

	//sprintf(smsg, "%s\nLine %d:\n%s")
	if (dynamic) {
		free((void*)smsg);
	}

	printf("%s", smsg);
}

Local<Value> JSState::ExecuteScript(Local<String> script, int *error) {
	EscapableHandleScope handle_scope(GetIsolate());

	// We're just about to compile the script; set up an error handler to
	// catch any exceptions the script might throw.
	TryCatch try_catch(GetIsolate());

	Local<Context> context(GetIsolate()->GetCurrentContext());

	Local<Value> result;

	if (error) *error = 0;

	// Compile the script and check for errors.
	Local<Script> compiled_script;
	if (!Script::Compile(context, script).ToLocal(&compiled_script)) {
		format_script_error(try_catch, GetIsolate());
		// The script failed to compile; bail out.

		if (error) *error = 1;
		result = try_catch.Exception();
	}

	// Run the script!
	if (!compiled_script->Run(context).ToLocal(&result)) {
		format_script_error(try_catch, GetIsolate());
		// Running the script failed; bail out.
		result = try_catch.Exception();
		if (error) *error = 1;
	}

	return handle_scope.Escape(result);
}
#endif
extern "C" {
EXPORT void JSTestPrint(const char *msg) {
	if (log_file) {
		fprintf(log_file, msg);
		fflush(log_file);
	}
}

EXPORT void JSSetLogFile(FILE *file) {
	log_file = file;
}

EXPORT void JSInit() {
#ifndef NO_V8
	state = new JSState();
#endif
}

EXPORT void JSDestroy() {
#ifndef NO_V8
	delete state;
	state = 0;
#endif
}

EXPORT void JSHandleEvents() {
	state->handleEvents();
}

EXPORT int JSHaveEvents() {
	return state->haveEvents();
}

EXPORT JSValue JSExec(const char *script, int *error) {
	Isolate::Scope isolate_scope(state->GetIsolate());
	HandleScope handle_scope(state->GetIsolate());
	Local<Context> context = state->GetContext();
	Context::Scope context_scope(context);

	Local<String> source;
	if (!String::NewFromUtf8(state->GetIsolate(), script, NewStringType::kNormal, static_cast<int>(strlen(script))).ToLocal(&source)) {
		return 0;
	}

	Local<Value> value = state->ExecuteScript(source, error);
	Persistent<Value> *ret = new Persistent<Value>(state->GetIsolate(), value);
	//JSValue cret = reinterpret_cast<JSValue>(ret);
	JSValue cret = (JSValue)ret;

	return cret;
}

EXPORT void JSFreeValue(JSValue value) {
	Persistent<Value> *p = reinterpret_cast<Persistent<Value>*>(value);
	delete p;
}

}