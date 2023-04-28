// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cos_theta_smoother_config.proto

#ifndef PROTOBUF_INCLUDED_cos_5ftheta_5fsmoother_5fconfig_2eproto
#define PROTOBUF_INCLUDED_cos_5ftheta_5fsmoother_5fconfig_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_protobuf_cos_5ftheta_5fsmoother_5fconfig_2eproto 

namespace protobuf_cos_5ftheta_5fsmoother_5fconfig_2eproto {
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
}  // namespace protobuf_cos_5ftheta_5fsmoother_5fconfig_2eproto
namespace apollo {
namespace planning {
class CosThetaSmootherConfig;
class CosThetaSmootherConfigDefaultTypeInternal;
extern CosThetaSmootherConfigDefaultTypeInternal _CosThetaSmootherConfig_default_instance_;
}  // namespace planning
}  // namespace apollo
namespace google {
namespace protobuf {
template<> ::apollo::planning::CosThetaSmootherConfig* Arena::CreateMaybeMessage<::apollo::planning::CosThetaSmootherConfig>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace apollo {
namespace planning {

// ===================================================================

class CosThetaSmootherConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:apollo.planning.CosThetaSmootherConfig) */ {
 public:
  CosThetaSmootherConfig();
  virtual ~CosThetaSmootherConfig();

  CosThetaSmootherConfig(const CosThetaSmootherConfig& from);

  inline CosThetaSmootherConfig& operator=(const CosThetaSmootherConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  CosThetaSmootherConfig(CosThetaSmootherConfig&& from) noexcept
    : CosThetaSmootherConfig() {
    *this = ::std::move(from);
  }

  inline CosThetaSmootherConfig& operator=(CosThetaSmootherConfig&& from) noexcept {
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
  static const CosThetaSmootherConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const CosThetaSmootherConfig* internal_default_instance() {
    return reinterpret_cast<const CosThetaSmootherConfig*>(
               &_CosThetaSmootherConfig_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(CosThetaSmootherConfig* other);
  friend void swap(CosThetaSmootherConfig& a, CosThetaSmootherConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline CosThetaSmootherConfig* New() const final {
    return CreateMaybeMessage<CosThetaSmootherConfig>(NULL);
  }

  CosThetaSmootherConfig* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<CosThetaSmootherConfig>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const CosThetaSmootherConfig& from);
  void MergeFrom(const CosThetaSmootherConfig& from);
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
  void InternalSwap(CosThetaSmootherConfig* other);
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

  // optional int32 print_level = 4 [default = 0];
  bool has_print_level() const;
  void clear_print_level();
  static const int kPrintLevelFieldNumber = 4;
  ::google::protobuf::int32 print_level() const;
  void set_print_level(::google::protobuf::int32 value);

  // optional bool ipopt_use_automatic_differentiation = 9 [default = false];
  bool has_ipopt_use_automatic_differentiation() const;
  void clear_ipopt_use_automatic_differentiation();
  static const int kIpoptUseAutomaticDifferentiationFieldNumber = 9;
  bool ipopt_use_automatic_differentiation() const;
  void set_ipopt_use_automatic_differentiation(bool value);

  // optional double weight_cos_included_angle = 1 [default = 10000];
  bool has_weight_cos_included_angle() const;
  void clear_weight_cos_included_angle();
  static const int kWeightCosIncludedAngleFieldNumber = 1;
  double weight_cos_included_angle() const;
  void set_weight_cos_included_angle(double value);

  // optional double weight_anchor_points = 2 [default = 1];
  bool has_weight_anchor_points() const;
  void clear_weight_anchor_points();
  static const int kWeightAnchorPointsFieldNumber = 2;
  double weight_anchor_points() const;
  void set_weight_anchor_points(double value);

  // optional double weight_length = 3 [default = 1];
  bool has_weight_length() const;
  void clear_weight_length();
  static const int kWeightLengthFieldNumber = 3;
  double weight_length() const;
  void set_weight_length(double value);

  // optional int32 max_num_of_iterations = 5 [default = 500];
  bool has_max_num_of_iterations() const;
  void clear_max_num_of_iterations();
  static const int kMaxNumOfIterationsFieldNumber = 5;
  ::google::protobuf::int32 max_num_of_iterations() const;
  void set_max_num_of_iterations(::google::protobuf::int32 value);

  // optional int32 acceptable_num_of_iterations = 6 [default = 15];
  bool has_acceptable_num_of_iterations() const;
  void clear_acceptable_num_of_iterations();
  static const int kAcceptableNumOfIterationsFieldNumber = 6;
  ::google::protobuf::int32 acceptable_num_of_iterations() const;
  void set_acceptable_num_of_iterations(::google::protobuf::int32 value);

  // optional double tol = 7 [default = 1e-08];
  bool has_tol() const;
  void clear_tol();
  static const int kTolFieldNumber = 7;
  double tol() const;
  void set_tol(double value);

  // optional double acceptable_tol = 8 [default = 0.1];
  bool has_acceptable_tol() const;
  void clear_acceptable_tol();
  static const int kAcceptableTolFieldNumber = 8;
  double acceptable_tol() const;
  void set_acceptable_tol(double value);

  // @@protoc_insertion_point(class_scope:apollo.planning.CosThetaSmootherConfig)
 private:
  void set_has_weight_cos_included_angle();
  void clear_has_weight_cos_included_angle();
  void set_has_weight_anchor_points();
  void clear_has_weight_anchor_points();
  void set_has_weight_length();
  void clear_has_weight_length();
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
  void set_has_ipopt_use_automatic_differentiation();
  void clear_has_ipopt_use_automatic_differentiation();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  ::google::protobuf::int32 print_level_;
  bool ipopt_use_automatic_differentiation_;
  double weight_cos_included_angle_;
  double weight_anchor_points_;
  double weight_length_;
  ::google::protobuf::int32 max_num_of_iterations_;
  ::google::protobuf::int32 acceptable_num_of_iterations_;
  double tol_;
  double acceptable_tol_;
  friend struct ::protobuf_cos_5ftheta_5fsmoother_5fconfig_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// CosThetaSmootherConfig

// optional double weight_cos_included_angle = 1 [default = 10000];
inline bool CosThetaSmootherConfig::has_weight_cos_included_angle() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void CosThetaSmootherConfig::set_has_weight_cos_included_angle() {
  _has_bits_[0] |= 0x00000004u;
}
inline void CosThetaSmootherConfig::clear_has_weight_cos_included_angle() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void CosThetaSmootherConfig::clear_weight_cos_included_angle() {
  weight_cos_included_angle_ = 10000;
  clear_has_weight_cos_included_angle();
}
inline double CosThetaSmootherConfig::weight_cos_included_angle() const {
  // @@protoc_insertion_point(field_get:apollo.planning.CosThetaSmootherConfig.weight_cos_included_angle)
  return weight_cos_included_angle_;
}
inline void CosThetaSmootherConfig::set_weight_cos_included_angle(double value) {
  set_has_weight_cos_included_angle();
  weight_cos_included_angle_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.CosThetaSmootherConfig.weight_cos_included_angle)
}

// optional double weight_anchor_points = 2 [default = 1];
inline bool CosThetaSmootherConfig::has_weight_anchor_points() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void CosThetaSmootherConfig::set_has_weight_anchor_points() {
  _has_bits_[0] |= 0x00000008u;
}
inline void CosThetaSmootherConfig::clear_has_weight_anchor_points() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void CosThetaSmootherConfig::clear_weight_anchor_points() {
  weight_anchor_points_ = 1;
  clear_has_weight_anchor_points();
}
inline double CosThetaSmootherConfig::weight_anchor_points() const {
  // @@protoc_insertion_point(field_get:apollo.planning.CosThetaSmootherConfig.weight_anchor_points)
  return weight_anchor_points_;
}
inline void CosThetaSmootherConfig::set_weight_anchor_points(double value) {
  set_has_weight_anchor_points();
  weight_anchor_points_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.CosThetaSmootherConfig.weight_anchor_points)
}

// optional double weight_length = 3 [default = 1];
inline bool CosThetaSmootherConfig::has_weight_length() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void CosThetaSmootherConfig::set_has_weight_length() {
  _has_bits_[0] |= 0x00000010u;
}
inline void CosThetaSmootherConfig::clear_has_weight_length() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void CosThetaSmootherConfig::clear_weight_length() {
  weight_length_ = 1;
  clear_has_weight_length();
}
inline double CosThetaSmootherConfig::weight_length() const {
  // @@protoc_insertion_point(field_get:apollo.planning.CosThetaSmootherConfig.weight_length)
  return weight_length_;
}
inline void CosThetaSmootherConfig::set_weight_length(double value) {
  set_has_weight_length();
  weight_length_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.CosThetaSmootherConfig.weight_length)
}

// optional int32 print_level = 4 [default = 0];
inline bool CosThetaSmootherConfig::has_print_level() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void CosThetaSmootherConfig::set_has_print_level() {
  _has_bits_[0] |= 0x00000001u;
}
inline void CosThetaSmootherConfig::clear_has_print_level() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void CosThetaSmootherConfig::clear_print_level() {
  print_level_ = 0;
  clear_has_print_level();
}
inline ::google::protobuf::int32 CosThetaSmootherConfig::print_level() const {
  // @@protoc_insertion_point(field_get:apollo.planning.CosThetaSmootherConfig.print_level)
  return print_level_;
}
inline void CosThetaSmootherConfig::set_print_level(::google::protobuf::int32 value) {
  set_has_print_level();
  print_level_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.CosThetaSmootherConfig.print_level)
}

// optional int32 max_num_of_iterations = 5 [default = 500];
inline bool CosThetaSmootherConfig::has_max_num_of_iterations() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void CosThetaSmootherConfig::set_has_max_num_of_iterations() {
  _has_bits_[0] |= 0x00000020u;
}
inline void CosThetaSmootherConfig::clear_has_max_num_of_iterations() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void CosThetaSmootherConfig::clear_max_num_of_iterations() {
  max_num_of_iterations_ = 500;
  clear_has_max_num_of_iterations();
}
inline ::google::protobuf::int32 CosThetaSmootherConfig::max_num_of_iterations() const {
  // @@protoc_insertion_point(field_get:apollo.planning.CosThetaSmootherConfig.max_num_of_iterations)
  return max_num_of_iterations_;
}
inline void CosThetaSmootherConfig::set_max_num_of_iterations(::google::protobuf::int32 value) {
  set_has_max_num_of_iterations();
  max_num_of_iterations_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.CosThetaSmootherConfig.max_num_of_iterations)
}

// optional int32 acceptable_num_of_iterations = 6 [default = 15];
inline bool CosThetaSmootherConfig::has_acceptable_num_of_iterations() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void CosThetaSmootherConfig::set_has_acceptable_num_of_iterations() {
  _has_bits_[0] |= 0x00000040u;
}
inline void CosThetaSmootherConfig::clear_has_acceptable_num_of_iterations() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void CosThetaSmootherConfig::clear_acceptable_num_of_iterations() {
  acceptable_num_of_iterations_ = 15;
  clear_has_acceptable_num_of_iterations();
}
inline ::google::protobuf::int32 CosThetaSmootherConfig::acceptable_num_of_iterations() const {
  // @@protoc_insertion_point(field_get:apollo.planning.CosThetaSmootherConfig.acceptable_num_of_iterations)
  return acceptable_num_of_iterations_;
}
inline void CosThetaSmootherConfig::set_acceptable_num_of_iterations(::google::protobuf::int32 value) {
  set_has_acceptable_num_of_iterations();
  acceptable_num_of_iterations_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.CosThetaSmootherConfig.acceptable_num_of_iterations)
}

// optional double tol = 7 [default = 1e-08];
inline bool CosThetaSmootherConfig::has_tol() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void CosThetaSmootherConfig::set_has_tol() {
  _has_bits_[0] |= 0x00000080u;
}
inline void CosThetaSmootherConfig::clear_has_tol() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void CosThetaSmootherConfig::clear_tol() {
  tol_ = 1e-08;
  clear_has_tol();
}
inline double CosThetaSmootherConfig::tol() const {
  // @@protoc_insertion_point(field_get:apollo.planning.CosThetaSmootherConfig.tol)
  return tol_;
}
inline void CosThetaSmootherConfig::set_tol(double value) {
  set_has_tol();
  tol_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.CosThetaSmootherConfig.tol)
}

// optional double acceptable_tol = 8 [default = 0.1];
inline bool CosThetaSmootherConfig::has_acceptable_tol() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void CosThetaSmootherConfig::set_has_acceptable_tol() {
  _has_bits_[0] |= 0x00000100u;
}
inline void CosThetaSmootherConfig::clear_has_acceptable_tol() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void CosThetaSmootherConfig::clear_acceptable_tol() {
  acceptable_tol_ = 0.1;
  clear_has_acceptable_tol();
}
inline double CosThetaSmootherConfig::acceptable_tol() const {
  // @@protoc_insertion_point(field_get:apollo.planning.CosThetaSmootherConfig.acceptable_tol)
  return acceptable_tol_;
}
inline void CosThetaSmootherConfig::set_acceptable_tol(double value) {
  set_has_acceptable_tol();
  acceptable_tol_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.CosThetaSmootherConfig.acceptable_tol)
}

// optional bool ipopt_use_automatic_differentiation = 9 [default = false];
inline bool CosThetaSmootherConfig::has_ipopt_use_automatic_differentiation() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void CosThetaSmootherConfig::set_has_ipopt_use_automatic_differentiation() {
  _has_bits_[0] |= 0x00000002u;
}
inline void CosThetaSmootherConfig::clear_has_ipopt_use_automatic_differentiation() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void CosThetaSmootherConfig::clear_ipopt_use_automatic_differentiation() {
  ipopt_use_automatic_differentiation_ = false;
  clear_has_ipopt_use_automatic_differentiation();
}
inline bool CosThetaSmootherConfig::ipopt_use_automatic_differentiation() const {
  // @@protoc_insertion_point(field_get:apollo.planning.CosThetaSmootherConfig.ipopt_use_automatic_differentiation)
  return ipopt_use_automatic_differentiation_;
}
inline void CosThetaSmootherConfig::set_ipopt_use_automatic_differentiation(bool value) {
  set_has_ipopt_use_automatic_differentiation();
  ipopt_use_automatic_differentiation_ = value;
  // @@protoc_insertion_point(field_set:apollo.planning.CosThetaSmootherConfig.ipopt_use_automatic_differentiation)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_cos_5ftheta_5fsmoother_5fconfig_2eproto
