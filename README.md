# geometry_coder_C

This repository contains the source code for the latest version of the geometry Point Cloud encoder Silhouette 3D, written in C++. 
A Matlab version of this coder can be found in https://github.com/pointcloud-unb/GeometryCoder . 
For more information about this encoder, please refer to our list of papers at the bottom of the page.

# Authors

This is the result of an ongoing work that started in 2018 at Universidade de Brasilia. Several people have worked with the Silhouette encoder:

- Eduardo Peixoto          (Professor at Universidade de Brasilia)
- Edil Medeiros            (Professor at Universidade de Brasilia)
- Otho T. Komatsu          (undergrad/MSc student at Universidade de Brasilia)
- Evaristo Ramalho         (MSc student at Universidade de Brasilia)
- Lucas M. Alves           (undergrad student at Universidade de Brasilia)
- Estevam Albuquerque      (undergrad student at Universidade de Brasilia)  
- Eduardo Lemos            (undergrad student at Universidade de Brasilia)
- Rodrigo Borba            (undergrad student at Universidade de Brasilia)
- Rodrigo Rosário          (undergrad student at Universidade de Brasilia)
- Davi Freitas             (MSc student at Universidade de Brasilia)
- Ricardo de Queiroz       (Professor at Universidade de Brasilia)

## Compiling

The code has been tested to run in both Linux and Windows enviroments. 
It should work on mac, but it has not been tested in this platform. 

### Compiling in Linux

If you're in Linux environment, in the root folder just run make:

`$ make clean && make`

### Compiling in Windows

The code has been tested compiling with VisualStudio 19, with x64 target console application and using the platform toolset "Visual Studio 2019 (v142)".

You need to create a solution from existing code, and then add both the include and src folders to the solution.

# Using the encoder/decoder

The code accepts inputs from both the command line and a config file. The parameters provided in the command line **will overwrite** the parameters contained in the `.cfg` file, but a config file must be passed and it must exist.

We provide examples for all three encoders in the Config File:
- /params/param_s3d.cfg     - This is for the S3D encoder
- /params/param_s3ds.cfg    - This is for the S3D-S encoder
- /params/param_s3dst.cfg   - This is for the S3D-ST encoder

The parameters found in the config file are:
- "InputFile = ./frame0000.ply"                    path to the input file
- "OutputFile = ./frame0000.bin"                   path to the encoder output file (compressed bitstream)
- "ReconstructedFile = ./rec_frame0000.ply"        path to the decoder output file (reconstructed point cloud)
- "Action = 0"                                     action for (0 - none, 1 - encode, 2 - decode)
- "Axis = 4"                                       axis for encoding (0 - none, 1 - X axis, 2 - Y axis, 3 - Z axis, 4 - All axis)
- "NThreads = 0"                                   number of threads: (1 << NThreads) used for partitioning and encoding S3D-S and S3D_ST
- "AlgorithmChoice = 0"                            Encoder choice (0 - S3D, 3 - S3D-S, 4 - S3D-ST)

In order to use the command line to apply custom parameters, it is mandatory to use flags accordingly to:

- "-c param.cfg" path to configuration file;
- "-i test.ply" path to input file;
- "-o test.bin" path to output file;
- "-r test_rec.ply" path to reconstructed file;
- "-enc"/"-dec" for encoding/decoding
- "- nthread 4" value for the number of threads (1 << nthread). 

Example of calling the S3D-ST with 8 threads:

`./S3D -enc -c param_s3dst.cfg -nthread 3`

Example of decoding a binary file in described in the parameter file. 

`./S3D -dec -c param_s3dst.cfg` 

Note that all the parameters needed are written in the bitstream (encoder choice, number of threads, encoded axis). The decoder just needs the path to the binary file and the reconstructed file.

# Papers

More information about the Silhouette encoder can be found in our published papers. 

- Rodrigo Rosário, Eduardo Peixoto, “Intra-Frame Compression of Point Cloud Geometry using Boolean Decomposition”, 2019 IEEE Visual Communications and Image Processing (VCIP), 2019, Sydney. 2019 IEEE Visual Communications and Image Processing (VCIP), 2019. p. 1-4. doi= https://doi.org/10.1109/VCIP47243.2019.8965783.
- Eduardo Peixoto. “Intra-Frame Compression of Point Cloud Geometry Using Dyadic Decomposition”, IEEE Signal Processing Letters, Vol 27, pp. 246-250, 2020. doi= https://doi.org/10.1109/LSP.2020.2965322.
- Eduardo Peixoto, Edil Medeiros, Evaristo Ramalho, “Silhouette 4d: An Inter-Frame Lossless Geometry Coder Of Dynamic Voxelized Point Clouds”, 2020 IEEE International Conference on Image Processing (ICIP), 2020, Abu Dhabi. 2020 IEEE International Conference on Image Processing (ICIP), 2020. p. 2691. doi= https://doi.org/10.1109/ICIP40778.2020.9190648.
- Davi R. Freitas, Eduardo Peixoto, Ricardo L. de Queiroz, Edil Medeiros, “Lossy Point Cloud Geometry Compression Via Dyadic Decomposition”,2020 IEEE International Conference on Image Processing (ICIP), 2020, Abu Dhabi. 2020 IEEE International Conference on Image Processing (ICIP), 2020. p. 2731. doi= https://doi.org/10.1109/ICIP40778.2020.9190910.
- Davi Freitas, Eduardo Peixoto, Ricardo L. de Queiroz, Edil Medeiros, “Estudo de Perdas para Codificação de Geometria de Nuvens de Pontos por Decomposição Diádica”, XXXVIII Simpósio Brasileiro de Telecomunicações e Processamento de Sinais (SBrT 2020), 2020, Florianópolis. Anais do XXXVIII Simp´osio Brasileiro de Telecomunica¸c˜oes e Processamento de Sinais (SBrT 2020), 2020. (in portuguese).
- Evaristo Ramalho, Eduardo Peixoto, Edil Medeiros, “Silhouette 4D With Context Selection: Lossless Geometry Compression of Dynamic Point Clouds”, IEEE Signal Processing Letters, Vol 28, pp. 1660-1664, 2021. doi= https://doi.org/10.1109/LSP.2021.3102525.
- Otho T. Komatsu, Edil Medeiros, Lucas M. Alves, Eduardo Peixoto, "Multithreaded Algorithms for Lossless Intra Compression of Point Cloud Geometry based on the Silhouette 3D Coder", 2023 IEEE Internation Conference on Image Processing (ICIP), _accepted for publication_.









