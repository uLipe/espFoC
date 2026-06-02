# Sourced before IDF connect gdbinit — one-shot backtrace + source at first stop (app_main).
set pagination off
define hook-stop
  echo \n=== espFoC debug stop ===\n
  where
  list
  define hook-stop
  end
end
