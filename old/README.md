# Old folder

In here, there are some codes I used thorughout the development process. Also, you'll find a directory name **WidowX**. This was the first Arduino Library, and it worked. The problem is that it only worked with Arduino 1.8.x. so, if you read the ArbotiX guide properly, you know that it only works with Arduino 1.0.6. So why did I do it with Arduino 1.8.x in the first place? Well, it worked and I don't know why. And it was all great until it didn't, so I had to make the library once again. 

However, although it was sad to find out it wasn't working, I managed to remove the *BasicLinearAlgebra.h* dependency (still, look at it, it is a good library). But no problem, the library that you can find in `Arduino Library > WidowX` works fine with Arduino 1.0.6 (last tested on 24/06/2021 ~~06/24/2021 for the americans that have no clue on how to write properly a date~~)

> Note: I say I managed, but in reality I had to. BLA does not work with Arduino 1.0.6