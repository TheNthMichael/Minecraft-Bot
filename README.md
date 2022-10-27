This is heavily a WIP, some of these requirements were a pain in the ass to setup and so I would not expect being able to run this as is until I set this project up for end users and add a way for users to setup keybinds, adjusting mouse speed, setup the roi for screen captures, and setting up tesserocr for faster ocr. Some parts of this code are poorly optimized due to setting up my own event queue for actions however ended up using dictionary collections to contain data for each event. On my machine it runs decently fast however you may have a different experience depending on how fast tesserocr and screen grabbing runs on your machine. This tool also uses pydirectinput for input and may at times steal your mouse and buttons away - This is obnoxious but I haven't added support for a quick exit keybind, just alt tab until you see the ursina game engine window and hit alt-f4.

# Pathfinding code
Pathfinding code was snabbed from https://github.com/mdeyo/d-star-lite
Thank you for the code as I was too lazy to understand the psuedo code from that 2002 paper. I will however have to add support for 3d pathfinding with d*-field or whatever the name was. I'll figure it out later.

{
    (24.0, -56.0, -8.0): endesite - (24.0, -56.0, -8.0),
    (24.0, -56.0, -9.0): crimson nylium - (24.0, -56.0, -9.0),
    (42.0, -54.0, -8.0): endesite - (42.0, -54.0, -8.0),
    (25.0, -56.0, -9.0): endesite - (25.0, -56.0, -9.0),
    (24.0, -56.0, -10.0): endesite - (24.0, -56.0, -10.0),
    (23.0, -56.0, -9.0): endesite - (23.0, -56.0, -9.0)
}
[
    (x: (25, -54, -8), theta: (0, 0)),
    (x: (25, -55, -8), theta: (0, 60)),
    (x: (25, -56, -8), theta: (0, 60)),
    (x: (25, -56, -9), theta: (0, 90)),
    (x: (26, -54, -9), theta: (-90, 0)),
    (x: (26, -55, -9), theta: (-90, 60)),
    (x: (26, -56, -9), theta: (-90, 60)),
    (x: (25, -54, -10), theta: (180, 0)),
    (x: (25, -55, -10), theta: (180, 60)),
    (x: (25, -56, -10), theta: (180, 60)),
    (x: (24, -54, -9), theta: (90, 0)),
    (x: (24, -55, -9), theta: (90, 60)),
    (x: (24, -56, -9), theta: (90, 60))
]
