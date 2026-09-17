#!/usr/bin/env bash
# Build the brain. No Gradle, no Maven, no downloads: javac and the JDK only.
#
#   bash java/build.sh
#
# TeamCode is compiled with --release 8 against the SDK shim ONLY, which is the check that
# keeps the porting promise honest: if it builds here it builds on a Control Hub.
set -e
cd "$(dirname "$0")/.."

# The JDK here is the Windows one even under Git Bash, so the classpath separator follows
# the JVM, not the shell.
case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*) SEP=';' ;;
  *) SEP=':' ;;
esac

rm -rf java/out java/out-teamcode
mkdir -p java/out java/out-teamcode

echo "[1/3] SDK shim (Java 8)"
javac --release 8 -nowarn -Xlint:-options -d java/out $(find java/shim/src -name '*.java')

echo "[2/3] teamcode (Java 8, shim only -- this is the deliverable)"
javac --release 8 -nowarn -Xlint:-options -d java/out-teamcode -cp java/out $(find java/teamcode/src -name '*.java')

echo "[3/3] bridge, simsdk, runner (modern Java allowed)"
javac -nowarn -d java/out -cp "java/out${SEP}java/out-teamcode" $(find java/bridge/src java/simsdk/src java/runner/src -name '*.java')

echo "[4/4] teamcode self-check"
java -cp "java/out${SEP}java/out-teamcode" sim.runner.SelfCheck

echo "ok. OpModes:"
java -cp "java/out${SEP}java/out-teamcode" sim.runner.Main --list
