Main Reactor
============

![Now with 100% less design flaws!](https://i.stack.imgur.com/PjA4r.jpg)

[Based on the great sharecam example!](https://github.com/meirm/sharecam/tree/master/version4)

Some day soon, we'll have docs in this spot!

Shared Mat
==========

SharedMat.h is a utility class for sharing video streams efficiently
between processes. See `shared_reader.cpp` and `shared_writer.cpp`
for usage examples.

To try it out, start `shared_source` to capture the video stream
to a shared buffer. Then start as many `shared_reader`s as you'd
like. Each one will display the same video stream. `shared_reader`
takes the name of a filter to apply to the video stream - the only
options are 'blur' and 'canny', like so `./shared_reader blur`.

### Known issues
If a reader process crashes while holding the mutex, the source will deadlock.
