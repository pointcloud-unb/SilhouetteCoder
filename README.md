# geometry_coder_C

# To compile

If you're in Linux environment, in the root folder just run make:

`$ make clean && make`

# Command Line

In order to use the command line to apply custom parameters, it is mandatory to use flags accordingly to:

- "-c param.cfg" path to configuration file;
- "-i test.ply" path to input file;
- "-o test.bin" path to output file;
- "-r test_rec.ply" path to reconstructed file;
- "-m 8" value for single mode operation (== 0 normal | >= 1 singlemode).

The configuration file needs to obey the following example file `param.cfg`:

`InputFile = test.ply`

`OutputFile = test.bin`

`ReconstructedFile = test_rec.ply`

`SingleMode = 0`

`Action = 0`

`Axis = 0`

where `Action: 0,1,2 (EMPTY_ACTION, ENCODE_ACTION, DECODE_ACTION)` 
respectively, `Axis: 0,1,2,3,4 (EMPTY_AXIS, X,Y,Z, ALL_AXIS)`, respectively.
Also, we have the different options of encoders choice: `Action: 0,1,2,3,4 (S3D algorithm, undefined, undefined, S3D-S, S3D-ST)`. For the algorithms S3D-S and S3D-ST 
we can choose the amount of threads used, where we have the relation 
$$Amount Of Threads = 2^{NThreads}$$, where NThreads is the param present in the 
configuration file.

Lastly, the parameters provided in the command line **will overwrite** the parameters contained in the `.cfg` file.

# Test suite 

To test the data structures and implementations, run compile the test:

`$ make clean && make test_name`

and then execute:

`$ ./tests/folder_name/test_name.x`

