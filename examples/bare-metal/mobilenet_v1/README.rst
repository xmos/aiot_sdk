############
MobileNet V1
############

This example application implements a `MobileNet V1 <https://arxiv.org/abs/1704.04861>`__ CNN architecture trained on the `ImageNet <http://www.image-net.org/>`__ dataset.  The model was trained with the following parameters:

- Classes = 10
- Alpha = 0.50
- Image size = 128x128

This example demonstrates how to place models in LPDDR and to recieve input data using `xscope`.  The application will attempt to classify an entity in the image and assign it to one of the following classes:

- tench
- goldfish
- great_white_shark
- tiger_shark
- hammerhead
- electric_ray
- stingray
- cock
- hen
- ostrich

This example also demonstrates how to to build the inference engine into a static library.

*********************
Building the firmware
*********************

Make a directory for the build.

    $ mkdir build
    $ cd build

Run cmake:

    $ cmake ../
    $ make

To install, run:

    $ make install

Running the firmware
====================

Running with hardware

    $ xrun --io --xscope --xscope-port localhost:10234 bin/mobilenet_v1.xe

Running with simulator

    $ xsim --xscope "-realtime localhost:10234" bin/mobilenet_v1.xe

Test images can be sent to the firmware using `xscope`.  Most RGB images should work.  The `test_image.py` script requires the following Python modules:

    $ pip install numpy
    $ pip install matplotlib
    $ pip install opencv-python

Sending a test image to the xcore.ai Explorer board using `xscope`.

    $ ./test_image.py path/to/image

********************
Optimizing the model
********************

Unoptimized and optimized models are included with the example.

First, be sure you have installed the XMOS AI Toolchain extensions.  If installed, you can optimize your model with the following command:

    $ xformer.py --analyze -par 5 model/model_quant.tflite model/model_xcore.tflite

Converting flatbuffer to source file
====================================

The following unix command will generate a C source file that contains the TensorFlow Lite model as a char array

    $ python ../../../tools/generate/convert_tflite_to_c_source.py --input model/model_xcore.tflite --header inference_engine/src/mobilenet_v1.h --source inference_engine/src/mobilenet_v1.c --variable-name mobilenet_v1_model