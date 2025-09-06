# ESP Simple 3D

This is a project which uses a port of the textured 3d rasterization engine from
[finger563/cs283-project](https://github.com/finger563/cs283project) that I
wrote a long time ago while in undergrad / early grad school. 

<img width="715" height="949" alt="image" src="https://github.com/user-attachments/assets/20464188-e295-4f61-bd72-6d075cb99742" />

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ESP Simple 3D](#esp-simple-3d)
  - [Build and Flash](#build-and-flash)
  - [Output](#output)
  - [Example output of the rendered 3D scene:](#example-output-of-the-rendered-3d-scene)
  - [Developing](#developing)
    - [Code style](#code-style)

<!-- markdown-toc end -->

https://github.com/user-attachments/assets/e9ab1b37-0bc5-4236-88be-c9c24fec27ad

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

<img width="950" height="2959" alt="CleanShot 2025-09-06 at 15 57 53" src="https://github.com/user-attachments/assets/bbd11b3f-cb63-483b-b36f-07aa9506bf76" />

## Example output of the rendered 3D scene:

https://github.com/user-attachments/assets/e9ab1b37-0bc5-4236-88be-c9c24fec27ad

<img width="715" height="949" alt="image" src="https://github.com/user-attachments/assets/20464188-e295-4f61-bd72-6d075cb99742" />
<img width="715" height="949" alt="image" src="https://github.com/user-attachments/assets/f007ae79-d2d8-42a0-ad8b-6a3e429d2344" />
<img width="715" height="949" alt="image" src="https://github.com/user-attachments/assets/15cbc7c7-8955-411f-92e9-07f1944347f2" />
<img width="715" height="949" alt="image" src="https://github.com/user-attachments/assets/bd507bdb-18aa-45e5-9b6b-a0636da2287e" />
<img width="715" height="949" alt="image" src="https://github.com/user-attachments/assets/d10ee059-dcfa-4da8-8ac0-1b5e6d3ff2c1" />
<img width="715" height="949" alt="image" src="https://github.com/user-attachments/assets/772ac5d3-6a08-4c7c-b84d-7239ee866ef1" />

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
