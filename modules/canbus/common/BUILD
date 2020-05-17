load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_common",
    srcs = ["canbus_gflags.cc"],
    hdrs = ["canbus_gflags.h"],
  copts = [  "-DMODULE_NAME=\\\"pz\\\"",  ],  
    deps = [
        "//external:gflags",
    ],
)

cpplint()
