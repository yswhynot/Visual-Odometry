a=0
for i in *.jpg; do
	if [ $((a % 2)) -eq 0 ]; then
		b=$((a/2))
		new=$(printf "%02dr.jpg" "$b") #02 pad to length of 2
		mv -- "$i" "$new"
	else
		b=$(((a-1)/2))
		new=$(printf "%02dl.jpg" "$b")
		mv -- "$i" "$new"
	fi

  let a=a+1
done
