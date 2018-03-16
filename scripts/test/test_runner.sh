#!/bin/sh
TESTS_DIR='build/tests'
TEST_FORMAT="*_test"
LOG_TESTS=1
LOG_PATH=$TESTS_DIR/log
TMP_OUTPUT=output.tmp
FAILED=0
ALL_PASSED=1

GREEN="\033[32m"
RED="\033[31m"
ENDTC="\033[0m"


strip_term_colors()
{
  SED_ARGS="s/\x1B\[([0-9]{1,2}(;[0-9]{1,2})?)?[m|K]//g"
  SED_FLAGS=-r

  cat $LOG_FILE | sed $SED_FLAGS $SED_ARGS > $LOG_FILE.processed
  mv $LOG_FILE.processed $LOG_FILE
}

print_passed()
{
  echo -e "${GREEN}PASSED!${ENDTC}"
}

print_failed()
{
  echo -e "${RED}FAILED!${ENDTC}"
  echo "--------------------------------------------------"
  cat $TMP_OUTPUT
  echo "--------------------------------------------------"
}

check_exit_status()
{
  FAILED=$?
}

analyze_test_results()
{
  if [ $? != 0 ]
  then
    print_failed
  else
    print_passed
  fi
}

log_test()
{
  mkdir -p $LOG_PATH
  LOG_FILE=$LOG_PATH/$(basename ${TEST}.log)
  cp $TMP_OUTPUT $LOG_FILE
}

run_test()
{
  # run test
  TEST_NAME=${TEST/\.\//}
  TEST_NAME=${TEST_NAME//-//}

  printf "TEST [$TEST_NAME] "
  $TEST > $TMP_OUTPUT 2>&1
  analyze_test_results

  # log test
  if [ $LOG_TESTS -eq 1 ]
  then
    log_test
    strip_term_colors
  fi
}

main()
{
  # run tests
  cd ${TESTS_DIR}
  TESTS=$(find . -type f -name "${TEST_FORMAT}")
  for TEST in $TESTS
  do
    run_test
  done

  # exit properly
  if [ $ALL_PASSED -eq 0 ]
  then
    exit 1;
  else
    exit 0;
  fi
}


# RUN
main
