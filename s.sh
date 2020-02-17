#!/usr/bin/env bash
BUILD_TARGETS=`bazel query 'kind(cc_.*,tests(//modules/common/configs/...))'`
echo $BUILD_TARGETS
function get_filename(){
  file=$1
  search=":"
  prefix=${file%%$search*}
  # echo ${#prefix}
  filename=${file#$prefix}
  echo ${filename#$search}
}

  bazel clean
  #generate_build_targets
  
  newdir=data/singleTestCoverage
  mkdir $newdir
#add this to collect test coverage for each test file.
  for b_target in $BUILD_TARGETS; do
    echo $b_target
    dir=$(get_filename $b_target)
    echo $dir
    mkdir $newdir/$dir

    bazel test -c dbg --config=coverage $b_target
    COV_DIR=data/cov
    rm -rf $COV_DIR
    files=$(find bazel-out/local-dbg/bin/ -iname "*.gcda" -o -iname "*.gcno" | grep -v external | grep -v third_party)
    for f in $files; do
      if [ "$f" != "${f##*cyber}" ]; then
          target="$COV_DIR/objs/cyber${f##*cyber}"
      else
          target="$COV_DIR/objs/modules${f##*modules}"
      fi
      mkdir -p "$(dirname "$target")"
      cp "$f" "$target"
    done
    lcov -t $dir --rc lcov_branch_coverage=1 --base-directory "/apollo/bazel-apollo" --capture --directory "$COV_DIR/objs" --output-file "$COV_DIR/conv.info"
    if [ $? -ne 0 ]; then
      fail 'lcov failed!'
    fi
    lcov --rc lcov_branch_coverage=1 --remove "$COV_DIR/conv.info" \
       "external/*" \
       "/usr/*" \
       "bazel-out/*" \
       "*third_party/*" \
       "tools/*" \
       -o $COV_DIR/stripped_conv.info
    genhtml $COV_DIR/stripped_conv.info --output-directory $COV_DIR/report
    #after collect, copy it to other place for further analyze.
    #filename=$(get_filename $target)
    mv $COV_DIR $newdir/$dir
    bazel clean
    #delete 
  done

