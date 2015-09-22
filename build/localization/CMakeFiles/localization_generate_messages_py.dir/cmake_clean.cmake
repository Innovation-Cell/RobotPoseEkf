FILE(REMOVE_RECURSE
  "CMakeFiles/localization_generate_messages_py"
  "/home/rishabh/localization/devel/lib/python2.7/dist-packages/localization/msg/_lp.py"
  "/home/rishabh/localization/devel/lib/python2.7/dist-packages/localization/msg/_lla.py"
  "/home/rishabh/localization/devel/lib/python2.7/dist-packages/localization/msg/_roboteq_msg.py"
  "/home/rishabh/localization/devel/lib/python2.7/dist-packages/localization/msg/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/localization_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
