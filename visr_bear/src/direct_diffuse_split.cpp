#include "direct_diffuse_split.hpp"

namespace bear {
DirectDiffuseSplit::DirectDiffuseSplit(const SignalFlowContext &ctx,
                                       const char *name,
                                       CompositeComponent *parent,
                                       const ConfigImpl &config,
                                       std::shared_ptr<Panner> panner_)
    : AtomicComponent(ctx, name, parent),
      panner(std::move(panner_)),
      num_objects(config.num_objects_channels),
      gains_in("gains_in",
               *this,
               pml::MatrixParameterConfig(panner->num_gains() * 2, config.num_objects_channels)),
      direct_gains_out("direct_gains_out",
                       *this,
                       pml::MatrixParameterConfig(panner->num_gains(), config.num_objects_channels)),
      diffuse_gains_out("diffuse_gains_out",
                        *this,
                        pml::MatrixParameterConfig(panner->num_gains(), config.num_objects_channels)),
      hv_in("hv_in", *this, pml::MatrixParameterConfig(4, config.num_objects_channels)),
      room_gains_out("room_gains_out",
                     *this,
                     pml::MatrixParameterConfig(panner->num_gains(), config.num_objects_channels)),
      bypass_gains_out("bypass_gains_out", *this, pml::MatrixParameterConfig(2, config.num_objects_channels))
{
}

void DirectDiffuseSplit::process()
{
  for (size_t object_i = 0; object_i < num_objects; object_i++) {
    const float room = hv_in.data()(0, object_i), binaural = hv_in.data()(3, object_i);
    for (size_t gain_i = 0; gain_i < panner->num_gains(); gain_i++) {
      const float direct = binaural * gains_in.data()(gain_i, object_i);
      direct_gains_out.data()(gain_i, object_i) = direct;
      diffuse_gains_out.data()(gain_i, object_i) =
          binaural * gains_in.data()(panner->num_gains() + gain_i, object_i);
      room_gains_out.data()(gain_i, object_i) = room * direct;
    }
    bypass_gains_out.data()(0, object_i) = hv_in.data()(1, object_i);
    bypass_gains_out.data()(1, object_i) = hv_in.data()(2, object_i);
  }
}

}  // namespace bear
