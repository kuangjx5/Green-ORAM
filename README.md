# Green ORAM

Green ORAM is a locality-aware ORAM primitive that exploits space locality of data in the physical memory for improving Path ORAM to obtain better performance while maintaining security. We implement the simulation of Green ORAM by revising the USIMM simulator. We provide two kinds of stash: single-path-stash and multi-path-stash in our implement.



## Environment

- Linux version 5.4.0-70-generic (buildd@lgw01-amd64-039)
- gcc version 7.5.0
- Ubuntu 7.5.0-3ubuntu1~18.04



## USIMM Simulator

USIMM is the Utah SImulated Memory Module, a cycle-accurate memory system simulator follows JEDEC DDR3-1600 specification. The USIMM simulator can be download in [this page](http://utaharch.blogspot.com/2012/02/usimm.html). We use the newest 1.3 version and mainly modify the `main.c` and `scheduler,c` in the `src` folder to implement Green ORAM.



## Dataset

We select the dataset used in 2012 memory scheduling championship (MSC). Each workload contains 500 million representative instructions selected from the PARSEC and commercial benchmark with a methodology similar to Simpoint. The whole dataset can be downloaded in [this page](https://www.cs.utah.edu/~rajeev/jwac12/results_table.html).



## Preprocess

The downloaded dataset should be extracted to the `input` folder in our codes. Then it can be used in simulation. 



We also provide two codes to test the extreme cases: the `transLocal.cpp` can transform the origin addresses in workload into consecutive addresses; the `transRandom.cpp` transforms origin addresses into random addresses. You can use these codes as follow:



```sh
$ g++ transLocal.cpp -o transLocal
$ ./transLocal workload_name > workload_name_Local
```



## Set Green ORAM mode

We provide function `get_read_num_oram_single()` for common 1-path-stash and `get_read_num_oram_multi()` for 3-path-stash. These two functions are also the entrance of the ORAM relative functions.



As default, we use the 3-path-stash and call `get_read_num_oram_multi()` in `main()`. You can call  `get_read_num_oram_single()` instead for common stash. You can also comment out this line for the basic simulation without ORAM.



## Set simulation instructions

The simulations instructions are set in file `run_GreenORAM`. In each instruction, you need to appoint the memory channel size, the input file and the output file. You can write an instruction as follow:



```sh
bin/usimm input/1channel.cfg input/black > output/black_GreenORAM
```



## Make and run

After the preparations above, you can compile the project and run the simulation with the following commands:



```sh
$ cd src
$ make clean
$ make
$ cd ..
$ ./run_GreenORAM
```
