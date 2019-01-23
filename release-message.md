# prototype-microbit-as-ultibo-peripheral

# Requires:
* a micro:bit computer and a usb cable
* an RPI3B or RPI3B+ with a power supply - the RPI Zero is not yet supported
* a computer that can write an sd card
* an sd card that you can erase - its contents will be destroyed

# Optional:
* usb keyboard
* an hdmi tv and an hdmi cable

# Steps:
* with the computer
    * format the sd card as FAT32
        * this destroys the current contents of the sd card
    * download the zip file
    * unzip it to the sd card
* with the computer
    * attach the microbit to the computer with the usb cable
    * the microbit should appear as a usb folder
    * copy microbit-blebuttonsbroadcaster.hex from the sd card to the microbit usb folder
    * safely eject the sd card
* insert the sd card into the pi
* connect the pi to the tv using the hdmi cable
* connect the pi to the optional usb keyboard
* turn on the tv
* apply power to the pi
* if using a tv, you should see a green border with large white regions with black text

# Operation:
* press the A and B buttons on the microbit - the activity should be reflected on the ultibo screen
