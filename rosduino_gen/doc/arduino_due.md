# Заголовки ROS 2 и заголовки для Arduino Due

`rosidl_generator_cpp` выдаёт C++-типы для узла на Linux/macOS (`rclcpp`). `rosduino-gen` выдаёт **другие** `.h`: те же имена ROS 2, но структура под прошивку Arduino Due (и остальные платы RosDuino). Файлы ROS 2 на МК подключать нельзя.

## Стандартные заголовки ROS 2

Генератор `rosidl_generator_cpp` кладёт типы в include-дерево пакета:

```cpp
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
```

Типичные свойства:

- расширение `.hpp`, имя файла в нижнем регистре (`bool.hpp`);
- C++14/17, libstdc++, исключения;
- поля: `std::string`, `std::vector<T>`, `std::array<T, N>`, `rosidl_runtime_cpp::BoundedVector`;
- `float64` → `double`;
- куча (`new` / аллокаторы STL);
- сеттеры `set__field()`, операторы `==`, typesupport для RMW/DDS;
- заголовок один, реализация и typesupport — отдельные единицы трансляции.

Такой тип живёт в процессе агента. На Arduino Due его нет: нет STL, нет RMW, нет этих `.hpp`.

## Заголовки RosDuino (Arduino Due)

Due — SAM3X8E, компилятор **gcc 4.8.3**, стандарт **C++11**. Ядро прошивки: без исключений, RTTI, STL, `new`/`malloc`, Arduino `String`. Поэтому генератор пишет только `.h` (без `.cpp`) в раскладке ROS 2, но с именами как в `.msg`:

```cpp
#include <std_msgs/msg/Bool.h>
#include <std_srvs/srv/SetBool.h>
```

На Due скетч использует короткие имена (`std_msgs::Bool`) и полные (`std_msgs::msg::Bool`).

### Что внутри файла

| | ROS 2 (`*.hpp`) | RosDuino на Due (`*.h`) |
|---|---|---|
| Язык | C++14/17, STL | C++11, `stdint.h`, без STL |
| Куча | да | нет: поля в объекте / на стеке |
| Имя типа | строка typesupport | `typeName()` → `rosduino::core::StrView` |
| Хеш типа | полный RIHS01 в typesupport | первые 8 байт RIHS01 в `typeHashBytes()` |
| Flash | не используется | на Due имя и хеш в **RAM** (не `PROGMEM`; `PROGMEM` — только AVR) |
| CDR | typesupport / Fast CDR | `serialize(CdrWriter&)` / `deserialize(CdrReader&)` в самой структуре |
| Размер | не ограничен заранее | `maxSerializedSize()` — константа времени компиляции |

Имя типа и хеш на Due — обычные `static const` в RAM: у SAM3X одна адресная шина, `PROGMEM` как на AVR нет. Ветки `#if defined(__AVR__)` в том же файле Due не берёт.

### Поля

Проводной CDR тот же, что у ROS 2 (XCDR1 LE). C++-отображение другое:

| В `.msg` / `.srv` | ROS 2 | Arduino Due |
|---|---|---|
| `bool`, целые | те же ширины | те же ширины (`bool`, `uint8_t` … `int64_t`) |
| `float32` | `float` | `float` |
| `float64` | `double` | `double` (на Due 8 Б; на AVR `double` 32-битный, CDR пишет 8-байтный IEEE-754) |
| `string` | `std::string` | исходящая — `const char*`; входящая — указатель в RX-буфер, живёт только в колбэке |
| `T[N]` | `std::array<T, N>` | массив C `T field[N]` |
| `T[]` / `T[<=N]` | `std::vector` / `BoundedVector` | `rosduino::Sequence<T, Cap>` (`Cap` из `.msg` или 8); лишнее отбрасывается, `truncated == true` |
| вложенный тип | `pkg::msg::Type` | `pkg::msg::Type` (тот же заголовок RosDuino) |
| константа | `static constexpr` | `static const` (gcc 4.8.3) |

Последовательности фиксированной ёмкости: Due не выделяет память под «ещё один элемент».

### Сервисы

В ROS 2 запрос и ответ — отдельные классы (`SetBool_Request`, `SetBool_Response`) плюс traits.

В RosDuino один тип:

```cpp
std_srvs::SetBool srv;
srv.request.data = true;
```

Вложенные `SetBool::Request` и `SetBool::Response` есть. Простой API скетча работает со структурой целиком, полный — с парой `(const Request&, Response&)`.

### Чего нет на Due

- `#include` заголовков `rclcpp` / `rosidl_runtime_cpp` / `*.hpp` из `/opt/ros`;
- `std::string`, `std::vector`, исключений, виртуальных таблиц в сущностях;
- неограниченных динамических массивов и строк.

Совместимость с ROS 2 — на проводе (имя типа, RIHS, CDR), не на уровне C++ ABI.

## Куда класть файлы

`rosduino-gen --all` очищает `generated/` и пишет комплект из `arduino_types.txt`. Чтобы добавить свои типы,
допишите пакет или `pkg/msg|srv/Name` в этот файл и запустите `--all` снова. В скетч заголовки не попадают,
пока каталоги не скопированы в `RosDuino/src/` (рядом с `rosduino/`). Это ручной шаг: генератор библиотеку не меняет.
