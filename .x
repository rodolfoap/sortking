execute(){
	./online data/PETS09-S2L1.csv

	# ./online data/ADL-Rundle-6/det.txt
}
build(){
	[ -d build/ ] && {
		pushd build &> /dev/null;
	} || {
		mkdir build;
		pushd build &> /dev/null;
		cmake .. -Wdev
	}
	make -j8; STATUS=$?
	popd &> /dev/null;
	[ $STATUS == 0 ] && echo [100%] $(ls -l online) || echo [ERROR] Compilation error.
}
case "$1" in
	"")
		[ -f online ] || build;
		execute
	;;
	d)	# Debug
		debug
	;;
	e)
		vip online.cpp Online* Hungarian.* KalmanTracker.*
		build;
		execute;
	;;
esac
