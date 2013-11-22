image-memory-sharer
===================

This set of programs receives images over ROS, stores them into shared memory, and allows any number of processes to read them.

Some system are severly limited in the amount of shared memory which can be allocated, so often the images will be transported in grayscale to meet those restrictions.

This package is not necessarily meant to be production-grade and it can be broken by running the publisher before the subscriber, but it demonstrates the concept
