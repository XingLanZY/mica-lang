# MicaLang

Mica is a small, dynamically typed scripting language written in portable C11. It compiles to
bytecode for its own register-based virtual machine, and brings its own generational garbage
collector, module system, standard library and C embedding API. The engine is about 26,800 lines
of C and depends on nothing beyond the C standard library.

```mica
import System.Math

class Vec2 {
    var x = 0
    var y = 0

    func init(self, x, y) { self.x = x; self.y = y }
    func operator+(self, o) { return Vec2(self.x + o.x, self.y + o.y) }
    get length(self) { return System.Math.sqrt(self.x * self.x + self.y * self.y) }
}

let a = Vec2(3, 4)
let b = Vec2(1, 2)

print("a       =", a.x, a.y)         # a       = 3 4
print("|a|     =", a.length)         # |a|     = 5
print("|a + b| =", (a + b).length)   # |a + b| = 7.2111025509279791
```

## Features

- **Bytecode compiler and register-based VM.** 90 opcodes in 32-bit fixed-width instructions, member
  access through inline caches, and a choice of computed-goto or switch dispatch.
- **Generational garbage collection.** Non-copying mark and sweep, incremental marking and sweeping,
  and debt-driven pacing so that collections follow allocation rather than a fixed threshold.
- **A compact value model.** Values are 8 bytes and NaN-boxed; numbers, strings, arrays,
  insertion-ordered maps, closures, classes and coroutines are all first-class.
- **Classes.** Inheritance, `get`/`set` properties, computed properties, static members and
  operator overloading, including indexed reads and writes.
- **Algebraic data types.** Enums whose members carry payloads, error sets, `match` with
  destructuring patterns, and `is` type tests that understand inheritance.
- **Error values instead of exceptions.** Errors are returned with `throw` and consumed with `try`,
  `try?`, `try!` or `catch`, and `defer` runs cleanup in last-in-first-out order on every exit path.
- **Coroutines.** `create`, `resume`, `status` and `cancel`, with clean-up semantics on close.
- **Modules.** Dotted namespaces resolved by name, selective imports, and compiled `.mlib` libraries
  and program assemblies that can also be loaded at run time.
- **Reflection.** `typeof` returns a `Type` carrying its base type, fields, methods and properties.
- **Strict where it helps.** Conditions must be real booleans, every binding must be initialised, and
  a bare expression is not a statement.
- **Embeddable.** A small C API with native functions, native classes bound to C structs with
  finalisers, and a continuation protocol for calling script code that may suspend.

## Building

Requires CMake 3.20 or newer and a C11 compiler. MSVC and Clang are both tested.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --target MicaLang
```

The interpreter is `build/MicaLang` on Unix and `build/MicaLang.exe` on Windows.

## Usage

```
MicaLang [options] [script [args]]
```

| Option | Meaning |
| --- | --- |
| `-e CODE` | Execute `CODE`. May be repeated; chunks share one runtime and run in order. |
| `-l MODULE` | Load `MODULE` before the program. May be repeated. |
| `-i` | Interactive session. A script given on the command line runs first. |
| `-I DIR` | Add `DIR` to the module search path. May be repeated. |
| `--stdlib DIR` | Use `DIR` as the standard library directory. |
| `-E` | Ignore the `MICA_PATH` and `MICA_STDLIB` environment variables. |
| `-v`, `--version` | Print version and build information. |
| `-h`, `--help` | Print usage. |
| `-` | Read the program from standard input. |
| `--` | Stop option handling; the rest are script arguments. |
| `-t`, `--tokens` | Dump the token stream. |
| `-a`, `--ast` | Dump the syntax tree after semantic analysis. |
| `-c`, `--disasm` | Dump disassembled bytecode for every function. |
| `--emit FILE` | Compile `FILE` into a `.mlib`: an entry program becomes a runnable assembly, a module file becomes an importable library. |
| `-o FILE` | Output path for `--emit`. |

```sh
MicaLang hello.mica                    # run a program
MicaLang hello.mica input.txt          # its arguments arrive in the global array `arg`
MicaLang -e 'print(6 * 7)'
MicaLang -l System.Math -e 'print(System.Math.sqrt(2))'
MicaLang -i                            # interactive session
MicaLang --emit app.mica -o app.mlib   # compile
MicaLang app.mlib                      # run a compiled assembly
```

The exit status is `0` on success, `1` when the program failed to compile or raised an error, and
`2` when the command line was wrong.

Modules are resolved by name in the search path, first as source and then as a compiled library, so
`import System.Math` looks for `System.Math.mica` and then `System.Math.mlib` in each search
directory. The engine reads `MICA_PATH` and `MICA_STDLIB` for the search path, and `MICA_PEEPHOLE=0`,
`MICA_GC=0` and `MICA_NO_REGALLOC=1` to switch off the optimiser, the collector and the register
allocator for measurement.

## Language

### Values and types

There is one number type; `/` is true division, `//` floors and `%` follows the sign of the divisor.

```mica
let n     = 42
let pi    = 3.14159
let hex   = 0xFF
let bin   = 0b1010
let big   = 1'000'000          # digit separators

let s1    = "text"
let s2    = 'text'
let raw   = """no escapes or interpolation here"""
let interp = "1 + 2 = ${1 + 2}"     # "1 + 2 = 3"

print(7 / 2, 7 // 2, -5 % 3)        # 3.5 3 1
print(typeof(3).name)               # number
```

Arrays are ordered and maps keep insertion order with string keys:

```mica
let arr = [1, 2, 3]
arr.push(4)
print(arr.length, arr.join("-"), arr.contains(2))     # 4 1-2-3-4 true

let person = { name: "mica", age: 1 }                 # unquoted keys are strings
person.age = 2
print(person.name, person.has("age"), person.keys().length)   # mica true 2
```

Conditions require actual booleans, which is a deliberate departure from most scripting languages:

```mica
if 0 == 0 { print("fine") }
# if 0 { }        -> runtime error: cannot use number as a boolean
```

### Control flow

```mica
if score >= 90 { print("A") } elif score >= 80 { print("B") } else { print("C") }

var i = 0
while i < 5 { i += 1 }

for i in 1..5 { }                  # inclusive range
for i in 10..0 step -2 { }         # with a step
for fruit in ["apple", "banana"] { }
for code, city in { CN: "Beijing", JP: "Tokyo" } { }

for i in 0..100 {
    if i % 2 == 0 { continue }
    if i > 7 { break }
}
```

`if`, `while` and `for` are also expressions: the value of a block is the value of its last
statement, and `${ ... }` is a block expression with its own scope.

```mica
let grade  = if score >= 90 { "A" } elif score >= 80 { "B" } else { "C" }
let last   = for i in 1..3 { i * 10 }                        # 30
let total  = ${ let base = 20; base + 22 }                   # 42
```

### Functions and closures

```mica
func add(a, b) { return a + b }
let mul = func(a, b) { return a * b }              # anonymous function
let square = (x) => x * x                          # lambda

func greet(name, greeting = "hello") { return greeting + ", " + name }

func sum_all(...) {                                # varargs
    var total = 0
    for v in ... { total += v }
    return total
}

func minmax(a, b) { if a < b { return a, b } return b, a }
let lo, hi = minmax(3, 9)                          # multiple returns

func configure(width, height, title) { return title + ": " + width + "x" + height }
configure(title = "window", width = 100, height = 50)   # named arguments, any order
```

Closure literals use `{ ... }` with implicit arguments, which makes short callbacks pleasant:

```mica
let doubler = { $0 * 2 }
let d = doubler(5)                                 # 10

func apply_to_all(arr, f) {
    var out = []
    for x in arr { out.push(f(x)) }
    return out
}

let doubled = apply_to_all([1, 2, 3]) { $0 * 2 }   # trailing closure: [2, 4, 6]
```

Lambdas capture the enclosing locals; each call of the enclosing function produces an independent
closure, and anonymous `func` functions do not capture.

```mica
func counter() {
    var count = 0
    return () => { count += 1; return count }
}

let c = counter()
print(c(), c(), c())      # 1 2 3
```

A read-only binding can be computed on access, including inside a class where it becomes a getter:

```mica
let double => 21 * 2
```

### Classes

Fields are declared in the class body, methods declare `self` explicitly, and methods without
`self` are `static` and called on the class.

```mica
class Animal {
    var name
    var sound
    func init(self, name, sound) { self.name = name; self.sound = sound }
    func speak(self) { return self.name + " says " + self.sound }
}

class Dog : Animal {
    func init(self, name) { super.init(name, "woof") }
}

class Vec2 {
    var x
    var y
    func init(self, x, y) { self.x = x; self.y = y }
    static func zero() { return Vec2(0, 0) }
}

print(Dog("rex").speak())     # rex says woof
print(Vec2.zero().x)          # 0
```

Properties and operator overloading:

```mica
class Circle {
    var radius
    func init(self, radius) { self.radius = radius }
    get area(self) { return 3 * self.radius * self.radius }
    set diameter(self, d) { self.radius = d // 2 }
}

let c = Circle(5)
print(c.area)       # 75
c.diameter = 20     # setter
print(c.area)       # 300
```

```mica
class Vec {
    var x
    var y
    func init(self, x, y) { self.x = x; self.y = y }
    func operator+(self, o) { return Vec(self.x + o.x, self.y + o.y) }
    func operator-(self) { return Vec(-self.x, -self.y) }
    func operator==(self, o) { return self.x == o.x && self.y == o.y }
    func operator[](self, i) { return i == 0 ? self.x : self.y }
}
```

### Enums, errors and matching

Enum members may carry payloads, and `match` destructures them:

```mica
enum Color { Red(r, g, b), Green, Blue }
enum Status { Ok = 10, Err = 20, Pending }

let c = Color.Red(255, 0, 0)
print(c.payload[0], Color.Green, Status.Pending)     # 255 0 21

let sum = match c {
    Color.Red(r, g, b) => r + g + b,
    Color.Green => -1,
    _ => -2,
}                                                    # 255
```

Errors are values, not exceptions:

```mica
error IoError {
    FileOpenError(line, message),
    Unknown
}

func divide(a, b) throws {
    if b == 0 { throw error("division by zero") }
    return a / b
}

let v1 = divide(10, 2) catch 0            # 5, or 0 on failure
let v2 = divide(10, 0) catch e {          # inspect the error object
    print("caught:", e.message)
    0
}
let v3 = try? divide(10, 0)               # null on failure
let v4 = try divide(10, 2)                # propagate to my caller

do {
    try divide(10, 0)
} catch e {
    print("handled:", e.message)
}
```

`defer` runs when the enclosing function exits, in last-in-first-out order:

```mica
func cleanup_demo() {
    defer { print("second") }
    defer { print("first") }
    print("body")
}
# body, first, second
```

### Coroutines

Any function containing `yield` becomes a coroutine and must be driven through the coroutine API.

```mica
func gen() {
    let a = yield 1            # the value passed to the next resume lands here
    yield a + 10
}

let co = coroutine.create(gen)
print(coroutine.resume(co, 0))     # 1
print(coroutine.resume(co, 5))     # 15
print(coroutine.status(co))        # suspended
coroutine.cancel(co)               # runs pending defers and closes the coroutine
```

### Modules

A module is a file that declares its own name; only `export`ed declarations are visible to
importers, and everything else stays internal.

```mica
export module App.Utils

import System.Math

export let VERSION = "1.0"
export func sqrt(x) { return System.Math.sqrt(x) }

func normalize(x) { return x < 0 ? -x : x }     # internal
```

```mica
import App.Utils                  # App.Utils.sqrt(2)
import System.Math.{ pow, clamp } # pow(2, 10) directly
```

An entry program is not a module: it contains either top-level statements or a `main()` function,
but not both, and a module cannot be executed directly.

### Reflection

```mica
class Point {
    var x
    var y
    func init(self, x, y) { self.x = x; self.y = y }
    func dist(self) { return self.x * self.x + self.y * self.y }
    static func zero() { return Point(0, 0) }
    get len(self) { return self.dist() }
}

let t = typeof(Point)
print(t.name, t.is_class)                # Point true
print(t.methods().length)                # 2
print(t.static_methods()[0].name)        # zero
print(t.fields()[0].name, t.properties()[0].name)   # x len

let p = Point(3, 4)
print(t.fields()[0].get(p))              # 3
print(t.methods()[1].invoke(p))          # 25
```

`Type` exposes `name`, `full_name`, `base_type`, the `is_*` predicates, `methods()`,
`static_methods()`, `fields()`, `properties()` and `get_class()`; `Assembly` exposes `types()`,
`get_type()` and `modules()` and can load a compiled `.mlib` at run time.

## Embedding

The public header is `mica.h`. A runtime holds the collector, the global table and the modules, and
any number of contexts share it; a context must be freed before its runtime.

```c
#include "mica.h"

int main(void) {
    mica_runtime *rt = mica_runtime_new();
    mica_runtime_add_search_path(rt, "modules");
    mica_context *ctx = mica_context_new(rt);

    if (mica_dostring(ctx, "print(6 * 7)", 11) != 0) {
        fprintf(stderr, "mica: %s (line %d)\n", mica_error_message(ctx), mica_error_line(ctx));
    }

    mica_context_free(ctx);
    mica_runtime_free(rt);
    return 0;
}
```

Native functions receive their arguments and write their results, returning the result count.
Classes, fields, methods, properties, constants and enums can all be injected into a module
namespace:

```c
static uint32_t native_triple(mica_context *ctx, mica_value *args, uint32_t argc, mica_value *res) {
    (void)ctx;
    if (argc < 1) return 0;
    res[0] = mica_int(mica_as_int(args[0]) * 3);
    return 1;
}

mica_register_native(rt, "triple", native_triple, 1);
mica_register_class(rt, "App", "Vec2");
mica_register_function(rt, "App", "version", native_version, 0);
```

A native class binds a C struct to a script class, with a finaliser for its resources:

```c
typedef struct { int x, y; } Point;

mica_native_class_def def = { "Point", sizeof(Point), point_finalizer };
mica_value klass = mica_new_native_class(rt, "App", &def);
mica_register_method(rt, klass, "sum", point_sum, 0, false);
```

For hosts that call script code which may suspend, `mica_yield` suspends without `longjmp` and
`mica_callk` registers a continuation that is invoked with the resume arguments if the callee
yields.

## Implementation notes

Source text goes through a lexer, a parser, a semantic pass and a code generator, and the generated
bytecode is then improved by a graph-colouring register allocator (control-flow graph, liveness,
interference, with consecutive register groups preserved for calls and aggregate construction) and a
rule-driven peephole pass that rewrites and deletes instructions to a fixed point while protecting
jump targets.

The collector is a generational, non-copying mark and sweep with tri-color and generational write
barriers, incremental marking and sweeping under per-slice budgets, and a byte debt model that
controls when the next cycle starts. The root set is explicit so that a minor collection does not
walk the permanent object chain.

A compiled program can be serialised into a single `.mlib` file holding its modules and entry
closure, with a versioned on-disk format; assembling an application and loading libraries at run
time both use it.

## Status

Mica is a personal research and engineering project, not production software. The interfaces are
still moving, and there are no releases yet: the only versioned artifact is the bytecode and
assembly format, which the compiler and the loader check on both ends.

Known gaps: there is no integer type, there is no just-in-time compiler, and the standard library is
small (mathematics and file input and output, with no time, randomness, process control or regular
expressions). Performance is in the same class as Lua 5.5 on the project's game-loop benchmark, and
varies noticeably with the compiler used.

## License

MIT. See [LICENSE](LICENSE).
