# azsp_path_follower_converter

This package contains node that is able to transform xy points to pathfollower's format and start fllowing the path 

<details>
<summary markdown="span">Table of contents</summary>
<!-- Table of contents made automatically by Markdown All in One extension
https://marketplace.visualstudio.com/items?itemName=yzhang.markdown-all-in-one -->

- [azsp\_path\_follower\_converter](#azsp_path_follower_converter)
  - [Project structre (optional)](#project-structre-optional)
  - [Topics](#topics)
    - [Publishers](#publishers)
    - [Subscibers](#subscibers)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Roadmap (optional)](#roadmap-optional)
  - [Contributor(s)](#contributors)

</details>

## Project structre (optional)
```
.
├── CMakeLists.txt
├── launch
├── nodes
│   └── 
├── package.xml
├── pyproject.toml
├── README.md
├── setup.py
├── src
│   └── azsp_path_follower_converter
│       ├── 
│       ├── __init__.py
│       └── 
       



```

## Topics
### Publishers
| topic name      | message type            | example message                                                                                     |
| --------------- | ----------------------- | --------------------------------------------------------------------------------------------------- |
|  |  |  |
|         |       |        |
### Subscibers
| topic name        | message type      | example message |
| ----------------- | ----------------- | --------------- |
|  |  |  |
## Installation

Project is based on [Cookiecutter](https://www.cookiecutter.io/) and [poetry](https://python-poetry.org/)
```shell
$ pip install -U cookiecutter
$ curl -sSL https://install.python-poetry.org | python3
```

## Usage

```shell
$ cd azsp_path_follower_converter
$ poetry shell
$ poetry install # if you want to lock dependenies
$ source ../../devel/setup.bash
```
Run tests
```shell
$ source ../../devel/setup.bash
# run integration tests
$ rostest azsp_path_follower_converter add_one.test
# run unit tests
$ pytest src/
```
## Roadmap (optional)

- [x] Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod
- [x] tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam
- [ ] quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. 
- [ ] Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat 
- [ ] nulla pariatur.
## Contributor(s)

[Oskar  Osinski](mailto:oskar.osinski@pimt.lukasiewicz.gov.pl)