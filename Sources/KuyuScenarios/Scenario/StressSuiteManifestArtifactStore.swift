import Foundation

public struct StressSuiteManifestArtifactStore: Sendable {
    public enum StoreError: Error, Sendable, Equatable {
        case missingManifest(String)
        case manifestEscapesArtifactRoot(String)
        case invalidManifest(String)
    }

    public init() {}

    @discardableResult
    public func write(
        _ manifest: StressSuiteManifest,
        to outputURL: URL,
        artifactRoot: URL? = nil
    ) throws -> URL {
        if let artifactRoot {
            try validateContained(outputURL, artifactRoot: artifactRoot)
        }
        try FileManager.default.createDirectory(
            at: outputURL.deletingLastPathComponent(),
            withIntermediateDirectories: true
        )
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        try encoder.encode(manifest).write(to: outputURL, options: [.atomic])
        return outputURL.standardizedFileURL
    }

    public func validatedManifest(
        at url: URL,
        artifactRoot: URL? = nil
    ) throws -> StressSuiteManifest {
        if let artifactRoot {
            try validateContained(url, artifactRoot: artifactRoot)
        }
        guard FileManager.default.fileExists(atPath: url.path) else {
            throw StoreError.missingManifest(url.path)
        }
        do {
            return try JSONDecoder().decode(
                StressSuiteManifest.self,
                from: Data(contentsOf: url)
            )
        } catch {
            throw StoreError.invalidManifest(url.path)
        }
    }

    private func validateContained(
        _ url: URL,
        artifactRoot: URL
    ) throws {
        let rootPath = artifactRoot
            .resolvingSymlinksInPath()
            .standardizedFileURL
            .path
        let path = resolvedFilePath(url)
        guard path.hasPrefix(rootPath + "/") else {
            throw StoreError.manifestEscapesArtifactRoot(path)
        }
    }

    private func resolvedFilePath(_ url: URL) -> String {
        let parentURL = url
            .deletingLastPathComponent()
            .resolvingSymlinksInPath()
            .standardizedFileURL
        return parentURL
            .appendingPathComponent(url.lastPathComponent, isDirectory: false)
            .standardizedFileURL
            .path
    }
}
