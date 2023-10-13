FOLDER="../.."


protoc --cpp_out=. osmap.proto
# TODO (Taijing): Perhaps softlink the files instead of using copy
cp osmap.pb.cc "${FOLDER}/src/"
cp src/Osmap.cpp "${FOLDER}/src/"

cp osmap.pb.h "${FOLDER}/include/"
cp include/Osmap.h "${FOLDER}/include/"
