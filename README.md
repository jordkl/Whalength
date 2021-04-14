Feb 2020

If v6 is not starting please download the 'private' folder from the repository as this was not included in the .zip files. Please paste it into the folder that contains the 'whalength.m' file.



November 2019

The bug in the 5% width calculation has been fixed in version 6.

To correct any previous 5% width measurements made in version 5 you can use these conversion factors for the 3 different systems. Each conversion factor was generated by measuring over 200 known lengths with both v5 and v6 (for a range of locations and orientations within  the image). The mean of the conversion factor between each of these measurements is given below for each system. Please use these to multiply any 5% width measurements made with the version 5 GUI (except for I1P photos) to get the correct widths.

I1P stills:
1.1994 (range 1.1890-1.2097, standard deviation = 0.0040)  

I2P photos:
0.8728 (range: 0.8659-0.8794, standard deviation = 0.0019) 

I2P stills:
1.2007 (range: 1.1950-1.2066, Standard deviation = 0.0019)


All other measurements (total length, widths etc), and any measurements for the I1P photo system were unaffected by this bug and do not need to be corrected.

For a standalone application (does not require Matlab, works on PC only) see: https://drive.google.com/file/d/1ThKkr3rMv9C8DEKXevU_FXECXIEp8_EC/view?usp=sharing
 

# Whalength
Repository for the supplementary material of Frontiers article: Inexpensive Aerial Photogrammetry for Studies of Whales and Large Marine Animals (Front. Mar. Sci., 15 November 2017 | https://doi.org/10.3389/fmars.2017.00366).

- GUI code for measuring whales in drone images
- Instruction PDF for using GUI code
- Code to run the LiDAR/GPS datalogger
- Instruction PDF on building the LiDAR/GPS datalogger
