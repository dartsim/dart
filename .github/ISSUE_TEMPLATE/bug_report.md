---
name: Bug report
about: Create a report to help us improve
title: ''
labels: 'type: bug'
assignees: ''

---

> If you're not reporting a bug, please use the [forum](https://dartsim.discourse.group/) to ask questions.

### Bug Report
> Please answer the following questions for yourself before reporting a bug.
- [ ] I checked the [documentation](http://dartsim.github.io/) and the [forum](https://dartsim.discourse.group/) but found no answer.
- [ ] I checked to make sure that this issue has not already been filed.

#### Environment
> Select the following information.
* DART version: [e.g., master, 6.8.3]
* OS name and version name(or number): [e.g., Ubuntu 18.04, macOS Mojav, Windows 10]
* Compiler name and version number: [e.g., GCC 7.4.0, Clang 3.9.1]

#### Expected Behavior
> Please describe the behavior you are expecting.

#### Current Behavior
> What is the current behavior?
> If you have trouble in `cmake` or `make`, it would be very helpful to build DART with verbose option:
> `$ cmake .. -DDART_VERBOSE=On`
> `$ make VERBOSE=1`
> and then dump the results into a [gist](https://gist.github.com/) and share the link to gist here.

#### Steps to Reproduce
> Please provide detailed steps for reproducing the issue.
1. 
2. 
3. 
4. 

#### Code to Reproduce
> Please remember that with a working code it's easier to reproduce the bug and it's much faster to fix it.
> It would be great to fork this repo and create a regression test using [this template](https://github.com/dartsim/dart/blob/master/unittests/regression/test_Issue000Template.cpp).
