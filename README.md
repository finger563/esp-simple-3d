# ESP Simple 3D

This is a project which uses a port of the textured 3d rasterization engine from
[finger563/cs283-project](https://github.com/finger563/cs283project) that I
wrote a long time ago while in undergrad / early grad school. 

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ESP Simple 3D](#esp-simple-3d)
  - [Build and Flash](#build-and-flash)
  - [Output](#output)
  - [Developing](#developing)
    - [Code style](#code-style)

<!-- markdown-toc end -->

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Output

Example screenshot of the console output from this app:

## Developing

If you're developing code for this repository, it's recommended to configure
your development environment:

### Code style

1. Ensure `clang-format` is installed
2. Ensure [pre-commit](https://pre-commit.com) is installed
3. Set up `pre-commit` for this repository:

  ``` console
  pre-commit install
  ```

This helps ensure that consistent code formatting is applied, by running
`clang-format` each time you change the code (via a git pre-commit hook) using
the [./.clang-format](./.clang-format) code style configuration file.

If you ever want to re-run the code formatting on all files in the repository,
you can do so:

``` console
pre-commit run --all-files
```
