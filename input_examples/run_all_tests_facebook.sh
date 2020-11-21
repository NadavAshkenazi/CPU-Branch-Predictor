#!/bin/bash

passed=0
num_of_test=60
if [[ -f report.txt ]];then
    rm report.txt
fi

for filename in tests/test*.in; do
	#get test number
    test_num=`echo $filename | cut -d'.' -f1`
    #convert to unix
    dos2unix ${filename} &> /dev/null
    #run the BP and create output file
    #./bp_main $filename > ${test_num}Yours_.out
    #print the diff result side by side in the log file

   # if [[ -f ${test_num}Diff_Result.log ]];then
    #    rm ${test_num}Diff_Result.log
    #fi
    
   # echo ${test_num} diff ---------- results ---------- > ${test_num}Diff_Result.log
    diff -y --suppress-common-lines ${test_num}Yours.out ${test_num}Yours_.out >> ${test_num}Diff_Result.log
    #count the number of tests that passed
    var=`diff ${test_num}Yours.out ${test_num}Yours_.out | wc -l`
    if (( var == 0));then
    	let passed=passed+1
        test_num=`echo $test_num | cut -c11-12`
        echo test $test_num passed! >> report.txt
    fi
done

sort -n -k2 report.txt
echo " "
echo $passed tests out of $num_of_test