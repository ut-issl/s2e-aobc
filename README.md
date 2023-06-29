# S2E-AOBC
 [![check format](https://github.com/ut-issl/s2e-aobc/actions/workflows/check-format.yml/badge.svg)](https://github.com/ut-issl/s2e-aobc/actions/workflows/check-format.yml)
[![Build](https://github.com/ut-issl/s2e-aobc/actions/workflows/build.yml/badge.svg)](https://github.com/ut-issl/s2e-aobc/actions/workflows/build.yml)

## Overview

- S2E-AOBC is the user-side repository of the S2E for the AOCS module developed by ISSL/UT, Seiren, and JAXA.
- Support platforms
  - Windows Visual Studio 2022 C++ compiler with 32bit build 
  - Linux g++ compiler version 11 with 32bit build
  - We use [GitHub Actions](https://github.com/ut-issl/s2e-aobc/actions) to check the build errors in these compilers continuously.
- How to use
  - `The main developers` of the AOCS module directly use this repository to add new features and improve the module.
  - `General users` of the AOCS module do not need to use and edit this repository directly. They need to create a project-specific repository and define spacecraft-specific parameters within the repository.


## Documents

- [s2e-document](https://github.com/ut-issl/s2e-documents)


## Release style

- We use [Semantic Versioning 2.0.0](https://semver.org/) as the versioning style
  - Basic version format is `<major>.<minor>.<patch>`(like `4.0.0`)
  - Public API is declared in the code itself (currently, there is no definitive list)
- All releases should be tagged as `v<semver>`(like `v4.0.0`)


## For general users
### How to make a project-specific repository

- We recommend making a project-specific repository named `s2e-aobc-hoge-satellite`.
- Users can refer to the `s2e-aobc/example` directory to make a directory construction of `s2e-aobc-hoge-satellite`.
  - **NOTE** Please rewrite the `example` to your project name.
  ```
  - s2e-aobc (git submodule)
    - We recommend to use a released version of s2e-aobc.
  - data
  - CMakeLists.txt
  - CMakeSettings.json
  - README.md
  ```
- `CMakeLists.txt`
  - Please change the word `example` to suit your project name.

### How to build and execute the project-specific repository

- Please check the `README.md` in the example directly.

### How to join the development of this repository

- When general users add new features or remove this repository's bugs, please make upstream pull requests from a forked repository.
- Before making PRs, please carefully read the following `Development style`.
- If you have any questions, please feel free to ask us.

## For main developers
### How to construct the repository

- `git submodule`
  - This repository includes [s2e-core](https://github.com/ut-issl/s2e-core) with the `git submodule`. Please use the following commands to construct the directory.
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
  - Users can use `s2e-aobc/s2e-core/ExtLibraries/CMakeLists.txt` to download the external libraries.
  - Please find how to download the `ExtLibraries` in the [s2e-document](https://github.com/ut-issl/s2e-documents).
    - [How to build and execute with Visual Studio](https://github.com/ut-issl/s2e-documents/blob/develop/General/HowToCompileWithVisualStudio.md)
    - [How to compile with Ubuntu in Docker](https://github.com/ut-issl/s2e-documents/blob/develop/General/HowToCompileWithUbuntuInDocker.md)

### Clone Flight S/W repository and build

- Make the `FlightSW` directory at the same directory with `s2e-aobc`
- Clone the [C2A-AOBC](https://github.com/ut-issl/c2a-aobc) repository into `FlightSW`
  - Current support version: [v6.0.0](https://github.com/ut-issl/c2a-aobc/release/tag/v6.0.0)
- Directory Construction
  ```
  - s2e-aobc
  - FlightSW
    - c2a-aobc
  ```
- You can build `s2e-aobc` using `CMake` together with `c2a-aobc`.

### Development style

- Repository settings
  - Branch structure
    ```
    - main        # The latest operation guaranteed codes for general users
    - develop     # The latest buildable codes for S2E primary developers
    - feature/*   # Developing codes
    - hotfix/*    # Bug Fix codes
    ```
  - Push to `main` and `develop` is prohibited. All developers have to develop with `feature/*` or `hotfix/*` branch and make a pull request.

- Flow of development
  1. Make a `feature/*` branch from the `develop` branch.
     - To fix the small bugs in the latest release codes, please make `hotfix/*` branch from the `main` branch.
  2. Edit, commit, and push in the branch.
     - Please check the [coding convention](https://github.com/ut-issl/s2e-documents/blob/develop/General/CodingConvention.md) and the `code format` in the next section.
  3. Create a new pull request to the `develop` branch.
     - The target branch becomes the `main` branch for the `hotfix/*` branches.
  4. A maintainer reviews the pull request. If some problems are found, the maintainer proposes modifications.
  5. According to the maintainer's proposal, the developer modifies the codes and goes back to 3.
  6. The maintainer merges the `feature/*` branch with the `develop` branch.
  7. The code owners decide to merge the `develop` branch to the `main` branch and release a new version.

- Binary files
  - Binary file commit is prohibited.
  - Please write the link to such files, or make a script file to get the files.
  - **Exception**
    - Images for markdown document files are allowable when the file size is smaller than 200K Bytes.

- Code format
  - We use [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to format the source code.
  - We recommend installing the `clang-format` and format code before the commit. It also will be checked on CI.
  - Some modern editor has plugin/extension for code format. It will be very useful.
    - VSCode: [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
    - Vim: [vim-clang-format](https://github.com/rhysd/vim-clang-format)


## Brief history of development
### History
- 21st Apr. 2020: Development started in a private repository at GitLab.
- 26th Sep. 2022: Initial development was finished.
- 03rd Mar. 2023: Move to a private repository at GitHub to prepare to publish as OSS.
- xx Jun. 2023: Convert to a public repository.

### Contributors in the GitLab repository
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

## Publications
1. S. Ikari, and et al., "Development of Compact and Highly Capable Integrated AOCS Module for CubeSats", [2022-f-41](https://archive.ists.ne.jp/upload_pdf/F-9-05.pdf), 33rd ISTS, 2022.
