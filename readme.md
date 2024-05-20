# :video_camera: s3tcob :robot:

> "software that tracks things on a conveyor belt"
>
> Track objects from a video stream using image segmentation AI, and convert into real world
> 2D coordinates in real time.
>
> Part of University of Nottingham Mechanical Engineering (MEng) Group Design and Make Project (MMME4085).

The software was designed to be used for the group project, but other tracking tasks may be able
to be accomplished using this software.
It was designed primarily to be used on a Linux system, but any operating system that is capable of running
the Ultralytics Python library (and Python 3.11 at the time of writing) should be able to run the program
without issue.

## Requirements


1. Hardware:

   1. A reasonably powerful computer - my Dell XPS 9560 (i5, 8GB RAM) performs adequately for this project, but
      if running the software on higher resolution videos (above 640x480) or move advanced models.

      You may want to use a computer with a
      CUDA enabled graphics card (the graphics card should automatically be utilised if it is available and compatible).
   1. A high quality video source, such as a USB webcam.
      The higher quality the video source, the better.

      A good way to get a high quality video stream is to use a smartphone, which you likely already have anyway.
      The free [Droidcam](https://droidcam.app/) mobile app and desktop client work really well for this, but be sure to
      connect over USB, as the stream latency will likely be too high over WiFi.

1. Software:

   1. Likely any modern desktop operating system, although this has only been tested on Linux (EndeavourOS)
   1. Python 3.11 - Ultralytics will not run on newer versions! (as of 2024-05-20)
   1. The following python modules: `ultralytics numpy cv2`

1. A calibration checkerboard with a known constant physical location.

   You may find [this checkerboard](./calibration-directional-chessboard.pdf) works well.
   It has the following properties:

   - Dimensions: 3 by 9
   - Square size: 20 mm
   - A red square 10 mm grid for testing purposes
   - A picture of our adorable project mascot, Odette :elephant: :)

# Usage

0. Ensure that all requirements are present, including connecting and starting the desired video source
1. Run s3tcob:

   ```bash
   python src/s3tcob.py [path_to_stream]
   ```

   where `[path_to_stream]` will be something like `/dev/video0` (the Droidcam GUI tells you which path the stream is at)

2. Tune parameters.

   The software accepts a few options, all as command line parameters:

   - Model path (`-m PATH`, `--model_path PATH`) - The path to the trained segmentation model. If none is provided, it defaults to `yolov8n-seg.pt`, which will be downloaded if it is not present.
   - Padding (`-p`, `--padding PADDING`) - The padding (in millimetres) around the checkerboard of the transformed image,
     supplied as four comma separated integers in the order `left,top,right,bottom`.

     Padding values of `0,0,0,0` will show only the checkerboard and nothing else.

     You can also supply this individually using `--padding.[left/top/right/bottom] VALUE`
   - Pixels per mm in the output image (`--px-per-mm`)
   - Calibration board square size (`--board.square-mm`) - The size of squares in millimetres
   - Calibration board dimensions (`--board.dimensions.[x/y]`) - The size of the calibration board.
     Note that the size is not measured in squares, but the lines between squares.

     For example, the example calibration board with Odette has dimensions 3x9, although if you
     count the squares you will count 4x10.


# Program Flow

```mermaid
flowchart TD
TLS([Tracking Loop])
--> LOAD_ML[Load machine learning model]
--> CAP[Start video capture]
--> GET_FRAME[Get next frame of video]
--> AN_FRAME[Analyse frame]
--> SEND_RES[Send results to Transformer Loop]
--> GET_FRAME

TFXL([Transformer Loop])
--> GET_RES[Get next result from Tracking Loop]
--> TFX[Transform all points through calibrated matrix]
--> SEND_TFX[Send transformed points to Decider Loop]
--> GET_RES

MAIN([Main Loop])
--> FRAME_CALIB[Capture one frame from camera]
--> CALIB[Calibrate using frame]
--> INIT_TRACKER[Initialise and start Tracker Loop]
--> INIT_TFX[Initialise and start Transformer Loop]
--> NOOP[Do nothing]
--> NOOP
```
