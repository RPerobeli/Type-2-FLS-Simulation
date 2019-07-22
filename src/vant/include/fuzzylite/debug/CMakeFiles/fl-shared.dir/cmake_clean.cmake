file(REMOVE_RECURSE
  "bin/libfuzzylite-debug.pdb"
  "bin/libfuzzylite-debug.so"
  "bin/libfuzzylite-debug.so.6.0"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/fl-shared.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
