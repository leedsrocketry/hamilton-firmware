OS=$(uname)

if [ "$OS" == "Darwin" ]; then
    mono /Applications/Renode.app/Contents/MacOS/bin/Renode.exe --console -e s $1
elif [ "$OS" == "Linux" || "$OS" == "Windows"]; then
    renode --console -e s $1
else
    echo "Unsupported OS: $OS"
    exit 1
fi