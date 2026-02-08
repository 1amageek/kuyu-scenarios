// swift-tools-version: 6.2

import PackageDescription

let package = Package(
    name: "kuyu-scenarios",
    platforms: [
        .macOS(.v26)
    ],
    products: [
        .library(
            name: "KuyuScenarios",
            targets: ["KuyuScenarios"]
        ),
    ],
    dependencies: [
        .package(path: "../kuyu-core"),
        .package(path: "../kuyu-physics"),
        .package(url: "https://github.com/apple/swift-log", from: "1.9.1"),
        .package(url: "https://github.com/apple/swift-configuration", from: "1.0.2"),
    ],
    targets: [
        .target(
            name: "KuyuScenarios",
            dependencies: [
                .product(name: "KuyuCore", package: "kuyu-core"),
                .product(name: "KuyuPhysics", package: "kuyu-physics"),
                .product(name: "Logging", package: "swift-log"),
                .product(name: "Configuration", package: "swift-configuration"),
            ]
        ),
        .testTarget(
            name: "KuyuScenariosTests",
            dependencies: ["KuyuScenarios"]
        ),
    ]
)
