FILE(REMOVE_RECURSE
  "CMakeFiles/localization_generate_messages_cpp"
  "/home/rishabh/localization/devel/include/localization/lp.h"
  "/home/rishabh/localization/devel/include/localization/lla.h"
  "/home/rishabh/localization/devel/include/localization/roboteq_msg.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/localization_generate_messages_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
