OS=$(uname)

if [ "$OS" == "Darwin" ]; then
    mono /Applications/Renode.app/Contents/MacOS/bin/Renode.exe --console -e s $1
    # macOS-specific commands go here
elif [ "$OS" == "Linux" || "$OS" == "Windows"]; then
    # Further check if it is WSL (Windows Subsystem for Linux)
    renode --console -e s $1
else
    echo "Unsupported OS: $OS"
    exit 1
fi




