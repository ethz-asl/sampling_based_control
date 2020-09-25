### TODO 

- Some algo like `std::sort` should support execution policy in C++17 to enable parallelism
- All the discount factors can be compute ahead and cached to save computation and they are not even used
- Google style code formatting
- Reduce verbosity, e.g: replace `config_.rollouts` with `nk_`, `steps_` with `nt_` and so on.
- The same holds for all the `_array` quantities. Replace for example `uu` with something like `uu`, `xx` with `xx` and so fort 
- Unify the visualizer with the ros version or use this in all the examples so to not have dublicate tests with and without additional visualization