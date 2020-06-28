execute(){
	./online
}
debug(){
	nemiver online
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
