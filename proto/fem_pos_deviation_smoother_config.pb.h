// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: fem_pos_deviation_smoother_config.proto

#ifndef PROTOBUF_INCLUDED_fem_5fpos_5fdeviation_5fsmoother_5fconfig_2eproto
#define PROTOBUF_INCLUDED_fem_5fpos_5fdeviation_5fsmoother_5fconfig_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_fem_5fpos_5fdeviation_5fsmoother_5fconfig_2eproto 

namespace protobuf_fem_5fpos_5fdeviation_5fsmoother_5fconfig_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_fem_5fpos_5fdeviation_5fsmoother_5fconfig_2eproto
namespace apollo {
namespace planning {
class FemPosDeviationSmootherConfig;
class FemPosDeviationSmootherConfigDefaultTypeInternal;
extern FemPosDeviationSmootherConfigDefaultTypeInternal _FemPosDeviationSmootherConfig_default_instance_;
}  // namespace planning
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::planning::FemPosDeviationSmootherConfig* Arena::CreateMaybeMessage<::apollo::planning::FemPosDeviationSmootherConfig>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace planning {

// ===================================================================

class FemPosDeviationSmootherConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.planning.FemPosDeviationSmootherConfig) */ {
 public:
  FemPosDeviationSmootherConfig();
  virtual ~FemPosDeviationSmootherConfig();

  FemPosDeviationSmootherConfig(const FemPosDeviationSmootherConfig& from);

  inline FemPosDeviationSmootherConfig& operator=(const FemPosDeviationSmootherConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  FemPosDeviationSmootherConfig(FemPosDeviationSmootherConfig&& from) noexcept
    : FemPosDeviationSmootherConfig() {
    *this = ::std::move(from);
  }

  inline FemPosDeviationSmootherConfig& operator=(FemPosDeviationSmootherConfig&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const FemPosDeviationSmootherConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const FemPosDeviationSmootherConfig* internal_default_instance() {
    return reinterpret_cast<const FemPosDeviationSmootherConfig*>(
               &_FemPosDeviationSmootherConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(FemPosDeviationSmootherConfig* other);
  friend void swap(FemPosDeviationSmootherConfig& a, FemPosDeviationSmootherConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline FemPosDeviationSmootherConfig* New() const final {
    return CreateMaybeMessage<FemPosDeviationSmootherConfig>(NULL);
  }

  FemPosDeviationSmootherConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<FemPosDeviationSmootherConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const FemPosDeviationSmootherConfig& from);
  void MergeFrom(const FemPosDeviationSmootherConfig& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(FemPosDeviationSmootherConfig* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // optional bool apply_curvature_constraint = 5 [default = false];
  bool has_apply_curvature_constraint() const;
  void clear_apply_curvature_constraint();
  static const int kApplyCurvatureConstraintFieldNumber = 5;
  bool apply_curvature_constraint() const;
  void set_apply_curvature_constraint(bool value);

  // optional bool use_sqp = 8 [default = false];
  bool has_use_sqp() const;
  void clear_use_sqp();
  static const int kUseSqpFieldNumber = 8;
  bool use_sqp() const;
  void set_use_sqp(bool value);

  // optional bool verbose = 102 [default = false];
  bool has_verbose() const;
  void clear_verbose();
  static const int kVerboseFieldNumber = 102;
  bool verbose() const;
  void set_verbose(bool value);

  // optional int32 print_level = 200 [default = 0];
  bool has_print_level() const;
  void clear_print_level();
  static const int kPrintLevelFieldNumber = 200;
  ::google::protobuf::int32 print_level() const;
  void set_print_level(::google::protobuf::int32 value);

  // optional double time_limit = 101 [default = 0];
  bool has_time_limit() const;
  void clear_time_limit();
  static const int kTimeLimitFieldNumber = 101;
  double time_limit() const;
  void set_time_limit(double value);

  // optional double weight_fem_pos_deviation = 2 [default = 10000000000];
  bool has_weight_fem_pos_deviation() const;
  void clear_weight_fem_pos_deviation();
  static const int kWeightFemPosDeviationFieldNumber = 2;
  double weight_fem_pos_deviation() const;
  void set_weight_fem_pos_deviation(double value);

  // optional double weight_ref_deviation = 3 [default = 1];
  bool has_weight_ref_deviation() const;
  void clear_weight_ref_deviation();
  static const int kWeightRefDeviationFieldNumber = 3;
  double weight_ref_deviation() const;
  void set_weight_ref_deviation(double value);

  // optional double weight_path_length = 4 [default = 1];
  bool has_weight_path_length() const;
  void clear_weight_path_length();
  static const int kWeightPathLengthFieldNumber = 4;
  double weight_path_length() const;
  void set_weight_path_length(double value);

  // optional double weight_curvature_constraint_slack_var = 6 [default = 100];
  bool has_weight_curvature_constraint_slack_var() const;
  void clear_weight_curvature_constraint_slack_var();
  static const int kWeightCurvatureConstraintSlackVarFieldNumber = 6;
  double weight_curvature_constraint_slack_var() const;
  void set_weight_curvature_constraint_slack_var(double value);

  // optional double curvature_constraint = 7 [default = 0.2];
  bool has_curvature_constraint() const;
  void clear_curvature_constraint();
  static const int kCurvatureConstraintFieldNumber = 7;
  double curvature_constraint() const;
  void set_curvature_constraint(double value);

  // optional double sqp_ftol = 9 [default = 0.0001];
  bool has_sqp_ftol() const;
  void clear_sqp_ftol();
  static const int kSqpFtolFieldNumber = 9;
  double sqp_ftol() const;
  void set_sqp_ftol(double value);

  // optional double sqp_ctol = 10 [default = 0.001];
  bool has_sqp_ctol() const;
  void clear_sqp_ctol();
  static const int kSqpCtolFieldNumber = 10;
  double sqp_ctol() const;
  void set_sqp_ctol(double value);

  // optional int32 sqp_pen_max_iter = 11 [default = 10];
  bool has_sqp_pen_max_iter() const;
  void clear_sqp_pen_max_iter();
  static const int kSqpPenMaxIterFieldNumber = 11;
  ::google::protobuf::int32 sqp_pen_max_iter() const;
  void set_sqp_pen_max_iter(::google::protobuf::int32 value);

  // optional int32 sqp_sub_max_iter = 12 [default = 100];
  bool has_sqp_sub_max_iter() const;
  void clear_sqp_sub_max_iter();
  static const int kSqpSubMaxIterFieldNumber = 12;
  ::google::protobuf::int32 sqp_sub_max_iter() const;
  void set_sqp_sub_max_iter(::google::protobuf::int32 value);

  // optional int32 max_iter = 100 [default = 500];
  bool has_max_iter() const;
  void clear_max_iter();
  static const int kMaxIterFieldNumber = 100;
  ::google::protobuf::int32 max_iter() const;
  void set_max_iter(::google::protobuf::int32 value);

  // optional bool scaled_termination = 103 [default = true];
  bool has_scaled_termination() const;
  void clear_scaled_termination();
  static const int kScaledTerminationFieldNumber = 103;
  bool scaled_termination() const;
  void set_scaled_termination(bool value);

  // optional bool warm_start = 104 [default = true];
  bool has_warm_start() const;
  void clear_warm_start();
  static const int kWarmStartFieldNumber = 104;
  bool warm_start() const;
  void set_warm_start(bool value);

  // optional int32 max_num_of_iterations = 201 [default = 500];
  bool has_max_num_of_iterations() const;
  void clear_max_num_of_iterations();
  static const int kMaxNumOfIterationsFieldNumber = 201;
  ::google::protobuf::int32 max_num_of_iterations() const;
  void set_max_num_of_iterations(::google::protobuf::int32 value);

  // optional int32 acceptable_num_of_iterations = 202 [default = 15];
  bool has_acceptable_num_of_iterations() const;
  void clear_acceptable_num_of_iterations();
  static const int kAcceptableNumOfIterationsFieldNumber = 202;
  ::google::protobuf::int32 acceptable_num_of_iterations() const;
  void set_acceptable_num_of_iterations(::google::protobuf::int32 value);

  // optional double tol = 203 [default = 1e-08];
  bool has_tol() const;
  void clear_tol();
  static const int kTolFieldNumber = 203;
  double tol() const;
  void set_tol(double value);

  // optional double acceptable_tol = 204 [default = 0.1];
  bool has_acceptable_tol() const;
  void clear_acceptable_tol();
  static const int kAcceptableTolFieldNumber = 204;
  double acceptable_tol() const;
  void set_acceptable_tol(double value);

  // @@protoc_insertion_point(class_scope:apollo.planning.FemPosDeviationSmootherConfig)
 private:
  void set_has_weight_fem_pos_deviation();
  void clear_has_weight_fem_pos_deviation();
  void set_has_weight_ref_deviation();
  void clear_has_weight_ref_deviation();
  void set_has_weight_path_length();
  void clear_has_weight_path_length();
  void set_has_apply_curvature_constraint();
  void clear_has_apply_curvature_constraint();
  void set_has_weight_curvature_constraint_slack_var();
  void clear_has_weight_curvature_constraint_slack_var();
  void set_has_curvature_constraint();
  void clear_has_curvature_constraint();
  void set_has_use_sqp();
  void clear_has_use_sqp();
  void set_has_sqp_ftol();
  void clear_has_sqp_ftol();
  void set_has_sqp_ctol();
  void clear_has_sqp_ctol();
  void set_has_sqp_pen_max_iter();
  void clear_has_sqp_pen_max_iter();
  void set_has_sqp_sub_max_iter();
  void clear_has_sqp_sub_max_iter();
  void set_has_max_iter();
  void clear_has_max_iter();
  void set_has_time_limit();
  void clear_has_time_limit();
  void set_has_verbose();
  void clear_has_verbose();
  void set_has_scaled_termination();
  void clear_has_scaled_termination();
  void set_has_warm_start();
  void clear_has_warm_start();
  void set_has_print_level();
  void clear_has_print_level();
  void set_has_max_num_of_iterations();
  void clear_has_max_num_of_iterations();
  void set_has_acceptable_num_of_iterations();
  void clear_has_acceptable_num_of_iterations();
  void set_has_tol();
  void clear_has_tol();
  void set_has_acceptable_tol();
  void clear_has_acceptable_tol();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  bool apply_curvature_constraint_;
  bool use_sqp_;
  bool verbose_;
  ::google::protobuf::int32 print_level_;
  double time_limit_;
  double weight_fem_pos_deviation_;
  double weight_ref_deviation_;
  double weight_path_length_;
  double weight_curvature_constraint_slack_var_;
  double curvature_constraint_;
  double sqp_ftol_;
  double sqp_ctol_;
  ::google::protobuf::int32 sqp_pen_max_iter_;
  ::google::protobuf::int32 sqp_sub_max_iter_;
  ::google::protobuf::int32 max_iter_;
  bool scaled_termination_;
  bool warm_start_;
  ::google::protobuf::int32 max_num_of_iterations_;
  ::google::protobuf::int32 acceptable_num_of_iterations_;
  double tol_;
  double acceptable_tol_;
  friend struct ::protobuf_fem_5fpos_5fdeviation_5fsmoother_5fconfig_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// FemPosDeviationSmootherConfig

// optional double weight_fem_pos_deviation = 2 [default = 10000000000];
inline bool FemPosDeviationSmootherConfig::has_weight_fem_pos_deviation() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_weight_fem_pos_deviation() {
  _has_bits_[0] |= 0x00000020u;
}
inline void FemPosDeviationSmootherConfig::clear_has_weight_fem_pos_deviation() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void FemPosDeviationSmootherConfig::clear_weight_fem_pos_deviation() {
  weight_fem_pos_deviation_ = 10000000000;
  clear_has_weight_fem_pos_deviation();
}
inline double FemPosDeviationSmootherConfig::weight_fem_pos_deviation() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.weight_fem_pos_deviation)
  return weight_fem_pos_deviation_;
}
inline void FemPosDeviationSmootherConfig::set_weight_fem_pos_deviation(double value) {
  set_has_weight_fem_pos_deviation();
  weight_fem_pos_deviation_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.weight_fem_pos_deviation)
}

// optional double weight_ref_deviation = 3 [default = 1];
inline bool FemPosDeviationSmootherConfig::has_weight_ref_deviation() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_weight_ref_deviation() {
  _has_bits_[0] |= 0x00000040u;
}
inline void FemPosDeviationSmootherConfig::clear_has_weight_ref_deviation() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void FemPosDeviationSmootherConfig::clear_weight_ref_deviation() {
  weight_ref_deviation_ = 1;
  clear_has_weight_ref_deviation();
}
inline double FemPosDeviationSmootherConfig::weight_ref_deviation() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.weight_ref_deviation)
  return weight_ref_deviation_;
}
inline void FemPosDeviationSmootherConfig::set_weight_ref_deviation(double value) {
  set_has_weight_ref_deviation();
  weight_ref_deviation_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.weight_ref_deviation)
}

// optional double weight_path_length = 4 [default = 1];
inline bool FemPosDeviationSmootherConfig::has_weight_path_length() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_weight_path_length() {
  _has_bits_[0] |= 0x00000080u;
}
inline void FemPosDeviationSmootherConfig::clear_has_weight_path_length() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void FemPosDeviationSmootherConfig::clear_weight_path_length() {
  weight_path_length_ = 1;
  clear_has_weight_path_length();
}
inline double FemPosDeviationSmootherConfig::weight_path_length() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.weight_path_length)
  return weight_path_length_;
}
inline void FemPosDeviationSmootherConfig::set_weight_path_length(double value) {
  set_has_weight_path_length();
  weight_path_length_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.weight_path_length)
}

// optional bool apply_curvature_constraint = 5 [default = false];
inline bool FemPosDeviationSmootherConfig::has_apply_curvature_constraint() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_apply_curvature_constraint() {
  _has_bits_[0] |= 0x00000001u;
}
inline void FemPosDeviationSmootherConfig::clear_has_apply_curvature_constraint() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void FemPosDeviationSmootherConfig::clear_apply_curvature_constraint() {
  apply_curvature_constraint_ = false;
  clear_has_apply_curvature_constraint();
}
inline bool FemPosDeviationSmootherConfig::apply_curvature_constraint() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.apply_curvature_constraint)
  return apply_curvature_constraint_;
}
inline void FemPosDeviationSmootherConfig::set_apply_curvature_constraint(bool value) {
  set_has_apply_curvature_constraint();
  apply_curvature_constraint_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.apply_curvature_constraint)
}

// optional double weight_curvature_constraint_slack_var = 6 [default = 100];
inline bool FemPosDeviationSmootherConfig::has_weight_curvature_constraint_slack_var() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_weight_curvature_constraint_slack_var() {
  _has_bits_[0] |= 0x00000100u;
}
inline void FemPosDeviationSmootherConfig::clear_has_weight_curvature_constraint_slack_var() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void FemPosDeviationSmootherConfig::clear_weight_curvature_constraint_slack_var() {
  weight_curvature_constraint_slack_var_ = 100;
  clear_has_weight_curvature_constraint_slack_var();
}
inline double FemPosDeviationSmootherConfig::weight_curvature_constraint_slack_var() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.weight_curvature_constraint_slack_var)
  return weight_curvature_constraint_slack_var_;
}
inline void FemPosDeviationSmootherConfig::set_weight_curvature_constraint_slack_var(double value) {
  set_has_weight_curvature_constraint_slack_var();
  weight_curvature_constraint_slack_var_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.weight_curvature_constraint_slack_var)
}

// optional double curvature_constraint = 7 [default = 0.2];
inline bool FemPosDeviationSmootherConfig::has_curvature_constraint() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_curvature_constraint() {
  _has_bits_[0] |= 0x00000200u;
}
inline void FemPosDeviationSmootherConfig::clear_has_curvature_constraint() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void FemPosDeviationSmootherConfig::clear_curvature_constraint() {
  curvature_constraint_ = 0.2;
  clear_has_curvature_constraint();
}
inline double FemPosDeviationSmootherConfig::curvature_constraint() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.curvature_constraint)
  return curvature_constraint_;
}
inline void FemPosDeviationSmootherConfig::set_curvature_constraint(double value) {
  set_has_curvature_constraint();
  curvature_constraint_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.curvature_constraint)
}

// optional bool use_sqp = 8 [default = false];
inline bool FemPosDeviationSmootherConfig::has_use_sqp() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_use_sqp() {
  _has_bits_[0] |= 0x00000002u;
}
inline void FemPosDeviationSmootherConfig::clear_has_use_sqp() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void FemPosDeviationSmootherConfig::clear_use_sqp() {
  use_sqp_ = false;
  clear_has_use_sqp();
}
inline bool FemPosDeviationSmootherConfig::use_sqp() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.use_sqp)
  return use_sqp_;
}
inline void FemPosDeviationSmootherConfig::set_use_sqp(bool value) {
  set_has_use_sqp();
  use_sqp_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.use_sqp)
}

// optional double sqp_ftol = 9 [default = 0.0001];
inline bool FemPosDeviationSmootherConfig::has_sqp_ftol() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_sqp_ftol() {
  _has_bits_[0] |= 0x00000400u;
}
inline void FemPosDeviationSmootherConfig::clear_has_sqp_ftol() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void FemPosDeviationSmootherConfig::clear_sqp_ftol() {
  sqp_ftol_ = 0.0001;
  clear_has_sqp_ftol();
}
inline double FemPosDeviationSmootherConfig::sqp_ftol() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.sqp_ftol)
  return sqp_ftol_;
}
inline void FemPosDeviationSmootherConfig::set_sqp_ftol(double value) {
  set_has_sqp_ftol();
  sqp_ftol_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.sqp_ftol)
}

// optional double sqp_ctol = 10 [default = 0.001];
inline bool FemPosDeviationSmootherConfig::has_sqp_ctol() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_sqp_ctol() {
  _has_bits_[0] |= 0x00000800u;
}
inline void FemPosDeviationSmootherConfig::clear_has_sqp_ctol() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void FemPosDeviationSmootherConfig::clear_sqp_ctol() {
  sqp_ctol_ = 0.001;
  clear_has_sqp_ctol();
}
inline double FemPosDeviationSmootherConfig::sqp_ctol() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.sqp_ctol)
  return sqp_ctol_;
}
inline void FemPosDeviationSmootherConfig::set_sqp_ctol(double value) {
  set_has_sqp_ctol();
  sqp_ctol_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.sqp_ctol)
}

// optional int32 sqp_pen_max_iter = 11 [default = 10];
inline bool FemPosDeviationSmootherConfig::has_sqp_pen_max_iter() const {
  return (_has_bits_[0] & 0x00001000u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_sqp_pen_max_iter() {
  _has_bits_[0] |= 0x00001000u;
}
inline void FemPosDeviationSmootherConfig::clear_has_sqp_pen_max_iter() {
  _has_bits_[0] &= ~0x00001000u;
}
inline void FemPosDeviationSmootherConfig::clear_sqp_pen_max_iter() {
  sqp_pen_max_iter_ = 10;
  clear_has_sqp_pen_max_iter();
}
inline ::google::protobuf::int32 FemPosDeviationSmootherConfig::sqp_pen_max_iter() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.sqp_pen_max_iter)
  return sqp_pen_max_iter_;
}
inline void FemPosDeviationSmootherConfig::set_sqp_pen_max_iter(::google::protobuf::int32 value) {
  set_has_sqp_pen_max_iter();
  sqp_pen_max_iter_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.sqp_pen_max_iter)
}

// optional int32 sqp_sub_max_iter = 12 [default = 100];
inline bool FemPosDeviationSmootherConfig::has_sqp_sub_max_iter() const {
  return (_has_bits_[0] & 0x00002000u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_sqp_sub_max_iter() {
  _has_bits_[0] |= 0x00002000u;
}
inline void FemPosDeviationSmootherConfig::clear_has_sqp_sub_max_iter() {
  _has_bits_[0] &= ~0x00002000u;
}
inline void FemPosDeviationSmootherConfig::clear_sqp_sub_max_iter() {
  sqp_sub_max_iter_ = 100;
  clear_has_sqp_sub_max_iter();
}
inline ::google::protobuf::int32 FemPosDeviationSmootherConfig::sqp_sub_max_iter() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.sqp_sub_max_iter)
  return sqp_sub_max_iter_;
}
inline void FemPosDeviationSmootherConfig::set_sqp_sub_max_iter(::google::protobuf::int32 value) {
  set_has_sqp_sub_max_iter();
  sqp_sub_max_iter_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.sqp_sub_max_iter)
}

// optional int32 max_iter = 100 [default = 500];
inline bool FemPosDeviationSmootherConfig::has_max_iter() const {
  return (_has_bits_[0] & 0x00004000u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_max_iter() {
  _has_bits_[0] |= 0x00004000u;
}
inline void FemPosDeviationSmootherConfig::clear_has_max_iter() {
  _has_bits_[0] &= ~0x00004000u;
}
inline void FemPosDeviationSmootherConfig::clear_max_iter() {
  max_iter_ = 500;
  clear_has_max_iter();
}
inline ::google::protobuf::int32 FemPosDeviationSmootherConfig::max_iter() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.max_iter)
  return max_iter_;
}
inline void FemPosDeviationSmootherConfig::set_max_iter(::google::protobuf::int32 value) {
  set_has_max_iter();
  max_iter_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.max_iter)
}

// optional double time_limit = 101 [default = 0];
inline bool FemPosDeviationSmootherConfig::has_time_limit() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_time_limit() {
  _has_bits_[0] |= 0x00000010u;
}
inline void FemPosDeviationSmootherConfig::clear_has_time_limit() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void FemPosDeviationSmootherConfig::clear_time_limit() {
  time_limit_ = 0;
  clear_has_time_limit();
}
inline double FemPosDeviationSmootherConfig::time_limit() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.time_limit)
  return time_limit_;
}
inline void FemPosDeviationSmootherConfig::set_time_limit(double value) {
  set_has_time_limit();
  time_limit_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.time_limit)
}

// optional bool verbose = 102 [default = false];
inline bool FemPosDeviationSmootherConfig::has_verbose() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_verbose() {
  _has_bits_[0] |= 0x00000004u;
}
inline void FemPosDeviationSmootherConfig::clear_has_verbose() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void FemPosDeviationSmootherConfig::clear_verbose() {
  verbose_ = false;
  clear_has_verbose();
}
inline bool FemPosDeviationSmootherConfig::verbose() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.verbose)
  return verbose_;
}
inline void FemPosDeviationSmootherConfig::set_verbose(bool value) {
  set_has_verbose();
  verbose_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.verbose)
}

// optional bool scaled_termination = 103 [default = true];
inline bool FemPosDeviationSmootherConfig::has_scaled_termination() const {
  return (_has_bits_[0] & 0x00008000u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_scaled_termination() {
  _has_bits_[0] |= 0x00008000u;
}
inline void FemPosDeviationSmootherConfig::clear_has_scaled_termination() {
  _has_bits_[0] &= ~0x00008000u;
}
inline void FemPosDeviationSmootherConfig::clear_scaled_termination() {
  scaled_termination_ = true;
  clear_has_scaled_termination();
}
inline bool FemPosDeviationSmootherConfig::scaled_termination() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.scaled_termination)
  return scaled_termination_;
}
inline void FemPosDeviationSmootherConfig::set_scaled_termination(bool value) {
  set_has_scaled_termination();
  scaled_termination_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.scaled_termination)
}

// optional bool warm_start = 104 [default = true];
inline bool FemPosDeviationSmootherConfig::has_warm_start() const {
  return (_has_bits_[0] & 0x00010000u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_warm_start() {
  _has_bits_[0] |= 0x00010000u;
}
inline void FemPosDeviationSmootherConfig::clear_has_warm_start() {
  _has_bits_[0] &= ~0x00010000u;
}
inline void FemPosDeviationSmootherConfig::clear_warm_start() {
  warm_start_ = true;
  clear_has_warm_start();
}
inline bool FemPosDeviationSmootherConfig::warm_start() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.warm_start)
  return warm_start_;
}
inline void FemPosDeviationSmootherConfig::set_warm_start(bool value) {
  set_has_warm_start();
  warm_start_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.warm_start)
}

// optional int32 print_level = 200 [default = 0];
inline bool FemPosDeviationSmootherConfig::has_print_level() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_print_level() {
  _has_bits_[0] |= 0x00000008u;
}
inline void FemPosDeviationSmootherConfig::clear_has_print_level() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void FemPosDeviationSmootherConfig::clear_print_level() {
  print_level_ = 0;
  clear_has_print_level();
}
inline ::google::protobuf::int32 FemPosDeviationSmootherConfig::print_level() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.print_level)
  return print_level_;
}
inline void FemPosDeviationSmootherConfig::set_print_level(::google::protobuf::int32 value) {
  set_has_print_level();
  print_level_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.print_level)
}

// optional int32 max_num_of_iterations = 201 [default = 500];
inline bool FemPosDeviationSmootherConfig::has_max_num_of_iterations() const {
  return (_has_bits_[0] & 0x00020000u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_max_num_of_iterations() {
  _has_bits_[0] |= 0x00020000u;
}
inline void FemPosDeviationSmootherConfig::clear_has_max_num_of_iterations() {
  _has_bits_[0] &= ~0x00020000u;
}
inline void FemPosDeviationSmootherConfig::clear_max_num_of_iterations() {
  max_num_of_iterations_ = 500;
  clear_has_max_num_of_iterations();
}
inline ::google::protobuf::int32 FemPosDeviationSmootherConfig::max_num_of_iterations() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.max_num_of_iterations)
  return max_num_of_iterations_;
}
inline void FemPosDeviationSmootherConfig::set_max_num_of_iterations(::google::protobuf::int32 value) {
  set_has_max_num_of_iterations();
  max_num_of_iterations_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.max_num_of_iterations)
}

// optional int32 acceptable_num_of_iterations = 202 [default = 15];
inline bool FemPosDeviationSmootherConfig::has_acceptable_num_of_iterations() const {
  return (_has_bits_[0] & 0x00040000u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_acceptable_num_of_iterations() {
  _has_bits_[0] |= 0x00040000u;
}
inline void FemPosDeviationSmootherConfig::clear_has_acceptable_num_of_iterations() {
  _has_bits_[0] &= ~0x00040000u;
}
inline void FemPosDeviationSmootherConfig::clear_acceptable_num_of_iterations() {
  acceptable_num_of_iterations_ = 15;
  clear_has_acceptable_num_of_iterations();
}
inline ::google::protobuf::int32 FemPosDeviationSmootherConfig::acceptable_num_of_iterations() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.acceptable_num_of_iterations)
  return acceptable_num_of_iterations_;
}
inline void FemPosDeviationSmootherConfig::set_acceptable_num_of_iterations(::google::protobuf::int32 value) {
  set_has_acceptable_num_of_iterations();
  acceptable_num_of_iterations_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.acceptable_num_of_iterations)
}

// optional double tol = 203 [default = 1e-08];
inline bool FemPosDeviationSmootherConfig::has_tol() const {
  return (_has_bits_[0] & 0x00080000u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_tol() {
  _has_bits_[0] |= 0x00080000u;
}
inline void FemPosDeviationSmootherConfig::clear_has_tol() {
  _has_bits_[0] &= ~0x00080000u;
}
inline void FemPosDeviationSmootherConfig::clear_tol() {
  tol_ = 1e-08;
  clear_has_tol();
}
inline double FemPosDeviationSmootherConfig::tol() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.tol)
  return tol_;
}
inline void FemPosDeviationSmootherConfig::set_tol(double value) {
  set_has_tol();
  tol_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.tol)
}

// optional double acceptable_tol = 204 [default = 0.1];
inline bool FemPosDeviationSmootherConfig::has_acceptable_tol() const {
  return (_has_bits_[0] & 0x00100000u) != 0;
}
inline void FemPosDeviationSmootherConfig::set_has_acceptable_tol() {
  _has_bits_[0] |= 0x00100000u;
}
inline void FemPosDeviationSmootherConfig::clear_has_acceptable_tol() {
  _has_bits_[0] &= ~0x00100000u;
}
inline void FemPosDeviationSmootherConfig::clear_acceptable_tol() {
  acceptable_tol_ = 0.1;
  clear_has_acceptable_tol();
}
inline double FemPosDeviationSmootherConfig::acceptable_tol() const {
  // @@protoc_insertion_point(field_get:apollo.planning.FemPosDeviationSmootherConfig.acceptable_tol)
  return acceptable_tol_;
}
inline void FemPosDeviationSmootherConfig::set_acceptable_tol(double value) {
  set_has_acceptable_tol();
  acceptable_tol_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.FemPosDeviationSmootherConfig.acceptable_tol)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_fem_5fpos_5fdeviation_5fsmoother_5fconfig_2eproto
