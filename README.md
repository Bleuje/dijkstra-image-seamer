## dijkstra image seamer

Larger image from small image

![original grass image](/bin/data/grass.png)

![stitched grass images](/doc/result2.png)

![stitched rocks images with seam](/doc/result2-seam.png)

![original rocks image](/bin/data/rocks.jpg)

![stitched rocks images](/doc/result1.png)

![stitched rocks images with seam](/doc/result1-seam.png)

### comments

I didn't invent the algorithm idea, I got it from my maths teacher in 2011 or 2012.

This is a console-like use of openFrameworks. The program takes as arguments input and output image filenames. (on linux I run `make && ./bin/texture-path <input_image> <output_image> [--show-seam]`)

The algorithm is explained in the code file main.cpp.

The algorithm was used in one of my first coding projects in 2012-2013.
In 2019 I made a much better short uncommented version, with Qt. 
Today (December 22nd 2024) ChatGPT o1 translated it successfully for openFrameworks on first try and added explanations/comments.

The Qt code from November 2019: https://gist.github.com/Bleuje/91c22639d0f312b38a07d19ee032739a
