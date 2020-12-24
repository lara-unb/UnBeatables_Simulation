dirname=`dirname "$thisscript"`
if [ $dirname = "." ]; then
        dirname="$PWD"
fi

LD_LIBRARY_PATH=$dirname:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH