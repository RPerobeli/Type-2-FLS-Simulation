file(REMOVE_RECURSE
  "bin/libfuzzylite.pdb"
  "bin/libfuzzylite.so"
  "bin/libfuzzylite.so.6.0"
)

# Per-language clean rules from dependency scanning.
foreach(lang CXX)
  include(CMakeFiles/fl-shared.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
