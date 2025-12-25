grpc\protobuf\protoc --cpp_out=. dss.proto
grpc\protobuf\protoc --python_out=. dss.proto

#grpc\protobuf\protoc --grpc_out ./ --cpp_out ./ -I ./ --plugin=protoc-gen-grpc=grpc\grpc\grpc_cpp_plugin.exe dss.proto
#grpc\protobuf\protoc --grpc_out ./ --python_out ./ -I ./ --plugin=protoc-gen-grpc=grpc\grpc\grpc_python_plugin.exe dss.proto



