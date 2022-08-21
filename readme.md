# Multi Agent Pathfinding for Large Agents
### CMPT417 D100 Summer 2022 - Hang Ma
**Gaelan O'Shea-McKay** - 
301267831\
**Hassan Shahid** - 301382353\
**Bryan Wong** - 301381982

This project contains multiple solvers for the Multi-Agent Path Finding problem with large size (2x2 and 3x3) agents.

These solvers are implemented inside `code/code/mccbs.py` and can be invoked from the command line with:

`python run_experiments.py`

With a number of arguments.

**Required Arguments:**\
`--instance <file_path/file_name>` Specify the instance to attempt to solve.\
`--solver <solver_name> (Options: MCCBS, Prioritized)` Specify which solver to use.

**Optional Arguments:**\
`--batch` Run the solver against a batch of instances.\
`--splitter <splitter_name> (Options: standard, disjoint, symmetrical, asymmetrical)` Choose the splitting strategy to use. Defaults to `standard` if omitted.\
`--maxnodes <max_number_of_generated_nodes>` Choose a number of generated nodes at which to stop the solver. If omitted, will run until a solution is found or the computer runs out of memory.


It also contains a suite of instances for benchmarking the solvers, located inside `/code/code/benchmarking/`.

Some extra hard instances can also be found in `/code/code/benchmarking/hard/`

For example, to invoke the MCCBS solver with symmetrical splitting on the set of benchmarking instances and a limit of 500,000 generated nodes run:\
`cd code`\
`cd code`\
`python run_experiments.py --instance "benchmarking/inst_*" --solver MCCBS --splitter symmetrical --batch --maxnodes 500000`

The results of the solver are found in `/code/code/mccbs_<splitter_name>_results.csv`.