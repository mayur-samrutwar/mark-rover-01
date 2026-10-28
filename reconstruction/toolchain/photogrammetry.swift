// Native photogrammetry helper. Driver requests one OBJ detail plus native poses, bounds and sparse points.
import Foundation
import RealityKit
import Darwin
import simd

func emit(_ event: String, _ details: [String: Any] = [:]) {
    var object = details
    object["event"] = event
    object["time"] = ISO8601DateFormatter().string(from: Date())
    if let data = try? JSONSerialization.data(withJSONObject: object, options: [.sortedKeys]),
       let line = String(data: data, encoding: .utf8) { print(line); fflush(stdout) }
}
func saveJSON(_ value: Any, to url: URL) throws {
    let data = try JSONSerialization.data(withJSONObject: value, options: [.prettyPrinted, .sortedKeys])
    try data.write(to: url, options: .atomic)
}
func savePointCloud(_ cloud: PhotogrammetrySession.PointCloud, to url: URL) throws {
    FileManager.default.createFile(atPath: url.path, contents: nil)
    let file = try FileHandle(forWritingTo: url)
    defer { try? file.close() }
    let header = "ply\nformat ascii 1.0\nelement vertex \(cloud.points.count)\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nproperty uchar alpha\nend_header\n"
    try file.write(contentsOf: Data(header.utf8))
    var lines = ""
    for (i, point) in cloud.points.enumerated() {
        let p = point.position, c = point.color
        lines += "\(p.x) \(p.y) \(p.z) \(c.x) \(c.y) \(c.z) \(c.w)\n"
        if i % 4096 == 4095 { try file.write(contentsOf: Data(lines.utf8)); lines = "" }
    }
    if !lines.isEmpty { try file.write(contentsOf: Data(lines.utf8)) }
}
func array3(_ v: SIMD3<Float>) -> [Float] { [v.x, v.y, v.z] }
func matrixRows(_ m: simd_float4x4) -> [[Float]] {
    (0..<4).map { row in (0..<4).map { col in m[col][row] } }
}
func argument(_ name: String) -> String? {
    let args = CommandLine.arguments
    guard let i = args.firstIndex(of: name), i + 1 < args.count else { return nil }
    return args[i + 1]
}

@main struct AppleQuality {
    static func main() async {
        let supported = PhotogrammetrySession.isSupported
        let capability: [String: Any] = [
            "supported": supported,
            "maximumInputImageDimension": PhotogrammetrySession.limits.maximumInputImageDimension,
            "maximumNumberOfInputImages": PhotogrammetrySession.limits.maximumNumberOfInputImages,
            "defaultFeatureSensitivity": "normal", "defaultSampleOrdering": "unordered",
            "quality_profile_arguments": true,
            "isObjectMaskingEnabled": false,
            "ignoreBoundingBox_applied": false,
            "ignoreBoundingBox_reason": "Property absent from installed macOS14.4 SDK; not compiled or claimed enabled",
            "pose_export": "Native RealityKit pose transform, quaternion and translation; coordinate semantics not reinterpreted"
        ]
        emit("capability", capability)
        if CommandLine.arguments.contains("--probe") || CommandLine.arguments.count == 1 { return }
        guard supported else { emit("fatal", ["error":"PhotogrammetrySession.isSupported is false"]); exit(2) }
        guard CommandLine.arguments.contains("--allow-process"),
              let lease = argument("--run-id"), !lease.isEmpty,
              let input = argument("--input"), let output = argument("--output") else {
            emit("fatal", ["error":"Processing requires --allow-process --run-id ID --input DIR --output DIR"]); exit(2)
        }
        let detailName = argument("--detail") ?? "both"
        let formatName = argument("--formats") ?? "both"
        let orderingName = argument("--ordering") ?? "unordered"
        let sensitivityName = argument("--sensitivity") ?? "normal"
        guard ["unordered", "sequential"].contains(orderingName), ["normal", "high"].contains(sensitivityName) else {
            emit("fatal", ["error":"Unknown ordering or sensitivity value"]); exit(2)
        }
        guard ["full", "raw", "both"].contains(detailName), ["obj", "usdz", "both"].contains(formatName) else {
            emit("fatal", ["error":"Unknown detail or formats value"]); exit(2)
        }
        do {
            let outputURL = URL(fileURLWithPath: output, isDirectory: true)
            try FileManager.default.createDirectory(at: outputURL, withIntermediateDirectories: true)
            var config = PhotogrammetrySession.Configuration()
            config.featureSensitivity = sensitivityName == "normal" ? .normal : .high
            config.sampleOrdering = orderingName == "unordered" ? .unordered : .sequential
            config.isObjectMaskingEnabled = false
            config.checkpointDirectory = outputURL.appendingPathComponent("checkpoints", isDirectory: true)
            try FileManager.default.createDirectory(at: config.checkpointDirectory!, withIntermediateDirectories: true)
            let session = try PhotogrammetrySession(input: URL(fileURLWithPath: input, isDirectory: true), configuration: config)
            let details: [(String, PhotogrammetrySession.Request.Detail)] = detailName == "both" ? [("full", .full), ("raw", .raw)] : [(detailName, detailName == "raw" ? .raw : .full)]
            var requests: [PhotogrammetrySession.Request] = [.bounds, .poses, .pointCloud]
            var requestedFiles: [[String: String]] = []
            for (name, detail) in details {
                if formatName != "usdz" {
                    let url = outputURL.appendingPathComponent("obj-\(name)", isDirectory: true)
                    requests.append(.modelFile(url: url, detail: detail))
                    requestedFiles.append(["detail":name, "format":"obj-folder", "path":url.path])
                }
                if formatName != "obj" {
                    let url = outputURL.appendingPathComponent("scene-\(name).usdz")
                    requests.append(.modelFile(url: url, detail: detail))
                    requestedFiles.append(["detail":name, "format":"usdz", "path":url.path])
                }
            }
            try saveJSON(["capability":capability,"run_id":lease,"requested_files":requestedFiles,
                          "input":input,"output":output,"detail":detailName,"formats":formatName,
                          "featureSensitivity":sensitivityName,"sampleOrdering":orderingName],
                         to: outputURL.appendingPathComponent("session-config.json"))
            emit("starting", ["run_id":lease,"input":input,"requested_files":requestedFiles])
            let started = Date()
            var errors: [[String: String]] = []
            var skipped: [Int] = []
            var invalid: [[String: Any]] = []
            var downsampling = false
            var stitchingIncomplete = false
            var files: [String] = []
            try session.process(requests: requests)
            for try await event in session.outputs {
                switch event {
                case .inputComplete:
                    emit("inputComplete")
                case .requestProgress(let request, let fraction):
                    emit("requestProgress", ["request":String(describing:request),"fraction":fraction])
                case .requestProgressInfo(let request, let info):
                    var value: [String: Any] = ["request":String(describing:request)]
                    if let stage = info.processingStage { value["stage"] = String(describing:stage) }
                    if let remaining = info.estimatedRemainingTime { value["estimatedRemainingSeconds"] = remaining }
                    emit("requestProgressInfo", value)
                case .requestError(let request, let error):
                    let value = ["request":String(describing:request),"error":String(describing:error),"localized_error":error.localizedDescription]
                    errors.append(value); emit("requestError", value)
                case .invalidSample(let id, let reason):
                    let value: [String: Any] = ["sample_id":id,"reason":reason]
                    invalid.append(value); emit("invalidSample", value)
                case .skippedSample(let id):
                    skipped.append(id); emit("skippedSample", ["sample_id":id])
                case .automaticDownsampling:
                    downsampling = true; emit("automaticDownsampling")
                case .stitchingIncomplete:
                    stitchingIncomplete = true; emit("stitchingIncomplete")
                case .requestComplete(let request, let result):
                    switch result {
                    case .modelFile(let url):
                        files.append(url.path); emit("modelFile", ["request":String(describing:request),"path":url.path])
                    case .bounds(let bounds):
                        try saveJSON(["minimum":array3(bounds.min),"maximum":array3(bounds.max)], to:outputURL.appendingPathComponent("bounds.json"))
                        emit("bounds", ["minimum":array3(bounds.min),"maximum":array3(bounds.max)])
                    case .pointCloud(let cloud):
                        let path = outputURL.appendingPathComponent("pointcloud-native.ply")
                        try savePointCloud(cloud, to: path)
                        emit("pointCloud", ["count":cloud.points.count,"path":path.path,"per_point_image_ids_available":false])
                    case .poses(let poses):
                        let values: [[String: Any]] = poses.posesBySample.keys.sorted().map { id in
                            let p = poses.posesBySample[id]!
                            return ["sample_id":id,"input_url":poses.urlsBySample[id]?.path ?? "",
                                    "translation":array3(p.translation),
                                    "quaternion_xyzw":[p.rotation.imag.x,p.rotation.imag.y,p.rotation.imag.z,p.rotation.real],
                                    "native_transform_rows":matrixRows(p.transform.matrix)]
                        }
                        try saveJSON(["coordinate_note":"Native RealityKit Pose.transform; no OpenCV convention or metric-scale assertion", "poses":values], to:outputURL.appendingPathComponent("poses-native.json"))
                        emit("poses", ["count":values.count])
                    default:
                        emit("requestComplete", ["request":String(describing:request),"result":String(describing:result)])
                    }
                case .processingComplete:
                    let summary: [String: Any] = ["status":errors.isEmpty ? "complete" : "completed_with_errors",
                        "elapsed_seconds":Date().timeIntervalSince(started),"model_files":files,
                        "automatic_downsampling":downsampling,"skipped_sample_ids":skipped,
                        "invalid_samples":invalid,"stitching_incomplete":stitchingIncomplete,"request_errors":errors,
                        "collision_validated":false,"metric_scale_validated":false]
                    try saveJSON(summary, to:outputURL.appendingPathComponent("result.json"))
                    emit("processingComplete", summary)
                    if !errors.isEmpty { exit(1) }
                    return
                case .processingCancelled:
                    emit("processingCancelled"); exit(3)
                @unknown default:
                    emit("other", ["description":String(describing:event)])
                }
            }
            emit("fatal", ["error":"Outputs ended without processingComplete"]); exit(1)
        } catch {
            emit("fatal", ["error":String(describing:error),"localized_error":error.localizedDescription]); exit(1)
        }
    }
}
