# S2E_6U_AOCS
## Overview

- S2E_6U_AOCS is the S2E's user side repository for AOCS sub-system of ISSL 6U satellite project.
- This repository manages source codes and initialize files for simulation scenario definition since this is just a user repository. 

## Development style

- Basically, S2E_6U_AOCS developers should follow rules in [Gitlab_settings](https://gitlab.com/ut_issl/documents/gitlab_settings). When we have our own rule of development, it will be written here.

## Documents

- Documents for S2E_6U_AOCS are summarized in [S2E/Documents](https://gitlab.com/ut_issl/s2e/documents).
  - General documents for S2E should be summarized in  [S2E_Documents_OSS](https://github.com/ut-issl/s2e-documents).
  - The `Documents` repository is shared with `S2E_EQUULEUS` and other user repositories for ISSL projects.
  - Only ISSL members can access the repository.

## How to compile

- `S2E_6U_AOCS`ディレクトリと同じ階層に`s2e-core`と`ExtLibraries`を配置する
  - 別の名前にしたらCMakeListを編集する
  - [Github上のs2e-core](https://github.com/ut-issl/s2e-core)のdevelopブランチを使う
- `ExtLibraries`内に`cspice`をコピーする
  - `cspice`はひとまず [//nas4/share/project_data/ISSL_OSS/tmp/](file://nas4/share/project_data/ISSL_OSS/tmp/)に置いている
  - ウェブから落としてくる方法は後ほど追記する

## C2A Integration

- Make `FlightSW` directory at same directory with `S2E_6U_AOCS`
- Make `C2A` directory in `FlighSW` and clone a [C2A_ISSL6U_AOBC](https://gitlab.com/ut_issl/c2a/c2a_issl6u_aobc) repository
- Edit `S2E_6U_AOCS/CMakeLists.txt` as follows
  - `set(USE_C2A OFF)` -> `set(USE_C2A ON)`
- Build `S2E_6U_AOCS`
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