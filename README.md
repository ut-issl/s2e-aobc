# S2E-AOBC
## Overview

- S2E-AOBC is the S2E's user side repository for AOCS sub-system.
- This repository manages source codes and initialize files for simulation scenario definition since this is just a user repository. 

## Development style

- Basically, S2E-AOBC developers should follow rules in [Gitlab_settings](https://gitlab.com/ut_issl/documents/gitlab_settings). When we have our own rule of development, it will be written here.

## Documents

- [s2e-document](https://github.com/ut-issl/s2e-documents)

## How to compile

- `git submodule`
  - This repository include [s2e-core](https://github.com/ut-issl/s2e-core) with `gitsubmodule`. Please use the following commands to construct the directory.
    ```
    $ git clone git@github.com:ut-issl/s2e-aobc.git
    $ cd s2e-aobc/
    $ git submodule init
    $ git submodule update
    ```
  - Or use the following commands to clone the repository.
    ```
    $ git clone --recursive git@github.com:ut-issl/s2e-aobc.git
    ```
- External Libraries
  - Please execute `src-core/ExtLibraries/CMakeLists.txt` to download the external libraries.
    - Please see [s2e-document](https://github.com/ut-issl/s2e-documents) for more detailed information.

## C2A Integration

- Make `FlightSW` directory at same directory with `S2E-AOBC`
- Make `C2A` directory in `FlighSW` and clone a [C2A-AOBC](https://github.com/ut-issl/c2a-aobc) repository
- Edit `S2E-AOBC/CMakeLists.txt` as follows
  - `set(USE_C2A OFF)` -> `set(USE_C2A ON)`
- Build `S2E-AOBC`
- **Note:** When you add new c source file in C2A, you have to modify `C2A/CMakeLists.txt` directory

## Brief history of development
- 21st Apr. 2020: Development start in a private repository at GitLab.
- 26th Sep. 2022: Initial development was finished.
- 03rd Mar. 2023: Move to a private repository at GotHub to prepare publish as OSS

## Contributors in the GitLab repository
- ISSL, UT
  - Satoshi Ikari: 158 commits
  - Hirotaka Sekine: 47 commits
  - Toshio Imamura: 42 commits
  - Toshihiro Suzuki: 37 commits
  - Ryo Suzumoto: 20 commits
  - Takayuki Hosonuma: 14 commits
  - Masahiro Fujiwara: 13 commits
  - Nobuhiro Funabiki: 12 commits
  - Ryohei Takahashi: 4 commits
  - Toshifumi Igeta: 2 commits
  - Yoshinari Gyu: 1 commit
  - Keidai Iiyama: 1 commit