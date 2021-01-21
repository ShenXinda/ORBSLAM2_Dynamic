#!/bin/bash

workspace=$1  
datasetPath=$2
resultFileName=$3

workspace=${workspace%*/}"/"
datasetPath=${datasetPath%*/}"/"
resultFile=$workspace"evaluate/"$resultFileName
tempFile=$workspace"evaluate/temp.txt" # 保存执行evaluate_ate.py输出数据
tempFile2=$workspace"evaluate/temp2.txt" # 保存执行Examples/RGB-D/rgbd_tum输出数据

folders="$(ls "$datasetPath")"
cd "$workspace" || return #进入工作空间的目录
for foldername in $folders
do
	# >> 追加，> 覆盖
	echo "$foldername" >> "$resultFile"
	resultFloder=evaluate/$foldername
	mkdir "$resultFloder" 
	allFrames=$(awk 'END{print NR}' "$datasetPath$foldername""/associations.txt")
	
	# 每个数据集执行5次，计算均值
	for (( i=1; i<=5; i++ ))
	do
		command="Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt config/TUM3.yaml ""$datasetPath$foldername"" ""$datasetPath$foldername""/associations.txt"
		info=$(eval "$command") # 执行字符串指令
		echo "$info" >> "$tempFile2"
		timeLine=$(sed -n '/^mean tracking time/p' "$tempFile2")
		time=${timeLine##* }
		mv -i CameraTrajectory.txt "$resultFloder""/CameraTrajectory""$i"".txt"
		mv -i KeyFrameTrajectory.txt "$resultFloder""/KeyFrameTrajectory""$i"".txt"
		command="python2 ./evaluate/evaluate_ate.py ""$datasetPath$foldername""/groundtruth.txt ./evaluate/""$foldername""/CameraTrajectory""$i"".txt --plot ""$resultFloder""/""$foldername""_""$i"".png"
		eval "$command"
		command="python2 ./evaluate/evaluate_ate.py ""$datasetPath$foldername""/groundtruth.txt ./evaluate/""$foldername""/""CameraTrajectory""$i"".txt --verbose"
		error=$(eval "$command")
		trackOkFrame=$(awk 'END{print NR}' ./evaluate/"$foldername"/CameraTrajectory"$i".txt)
		echo "$error"" ""$trackOkFrame"" ""$allFrames"" ""$time" >> "$resultFile"
		echo "$error"" ""$trackOkFrame"" ""$allFrames"" ""$time" >> "$tempFile"
		rm "$tempFile2"
	done
	command="python ./evaluate/tool.py"
	error=$(eval "$command")
	echo "$error" >> "$resultFile"
	rm "$tempFile"
done
