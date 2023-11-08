#!/bin/bash
set -e

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

echo "Building"

/usr/bin/e2studio --launcher.suppressErrors -nosplash -application org.eclipse.cdt.managedbuilder.core.headlessbuild -data /app/workspace -import ${SCRIPTPATH} -cleanBuild firmware-avnet-rasyn || true &

pid=$! # Process Id of the previous running command
while kill -0 $pid 2>/dev/null
do
    echo "Still building..."
    sleep 2
done

wait $pid
if [ -f /app/workspace/firmware-avnet-rasyn/Debug/firmware-avnet-rasyn.srec ]; then
    echo "Building done"
    exit 0
else
    echo "Building failed"
    exit 1
fi
