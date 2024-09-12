# GPS-with-SquareLine-on-Lilygo-T4S3

This project is an effort by two friends working together.  One on the Squareline Studio coding and the other on the .ino coding to pull it all together and put the data on screen.  It was fun to do.  The two participants were Joe B. and Mike M.  Joe handled the SL and Mike wrote the .ino to drive it.  It was not easy at first but finally happened.  The real problem was that there was no design.  We just made it up as we went along since this was the first time we had worked with SL.  So we did not know what it could do or the limitations.  There were some holes, dug by SL that we fell in but finally found how to fill them in and continue.  There is a lot to be learned from this project.  We will try to upload both the SL project files and the exported, immediately compilable files so you can test it and then modify it for your use.  I hope you enjoy it and can use some of the techniques herewith to further your own project to greatness.  

Neither of us were compensated in any way, by any entity to produce this tool.  It was just a labor of love for programming and learning new things.

This project was developed on the Arduino IDE only.  There is just 1 ino code file and a couple dozen files exported from SquareLine Studio including screen definitions, font files, moon phase pictures and support code to glue it all together.

This was originally proposed as a way to learn how SquareLine Studio works with a LilyGo T4S3 and find if it was even possible.  There were SO many questions that needed to be answered.  It took quite a while to find all of the pieces that made it work properly but one of us (Joe B.) was persistent in his searches and did find those answers and that resulted in the project looking really nice and running great.  There are 3 screens, the main GPS page, a sky map of the satellites and a statistics page with location averaging.  Initially, they were selected with buttons but Joe got swipe working and my code supports it so swipe is the way to switch screens now.

On the first screen, you will find many fields showing various data bits that either come directly from the GPS or are calculated based on what it presents to my program.  Many of the fields have a hidden button that enables you to change the units or display style of the data in the box.  The second screen shows the satellites in the sky and the third screen has position averaging and shows other background information that the GPS unit delivers to the program.

We used an I2C GPS receiver from SparkFun.  There are several selections that you can make.  This one has QWICC connectors on it that match up with the T4S3 connector so connecting it was easy.  There are available 4-wire cables to enable one to just plug it in with no soldering required.  We used this model: https://www.sparkfun.com/products/14414

An initial, short video is here: https://www.youtube.com/watch?v=ijUpDqNJf68 -- A longer one will be posted soon and a link will be included here.

We just found an oddity.  Line 303 (amoled.XPowersPPM::disableStatLed();) compiles and runs for me but will not compile for Joe.  He is on the Arduino version 2 and has an error message but tried it on version 1 and still has the error message.  I am on IDE version 1 and it works just fine.  If it won't compile for you, just comment it out.  It is only there to turn off the irritating, red, flashing charging light.  It does not affect program operatioon.  The problem may be with board definition verion 3.0.4.  I am on 2.0.17, having had problems with 3.0.4, previously. More on this later.

So just compile the code and run it.  Click fields on the main screen to change units or visual presentation.  The setting are checked and changed ones are saved each 10 minutes on the modulo 10 minute.  If you shutdown or reboot the board before that, they will not be saved.  This was done to save wear on the flash memory.

We are open to all suggestiong and bug reports.  Thanks for checking out this utility program.  It has been fun developing it.  I hope you can make good use of the program or the techniques involved with making it run and interface with the T4S3.
