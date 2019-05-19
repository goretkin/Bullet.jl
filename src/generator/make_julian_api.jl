using MacroTools
using EponymTuples
import DataStructures: DefaultOrderedDict, OrderedDict

package_dir = joinpath(@__DIR__, "..", "..")
bullet_repo = joinpath(package_dir, "deps/usr/downloads/src/bullet3-c_api_refactor_build")
generated_dir = joinpath(package_dir, "src", "generated")

function parseall(str)
  return Meta.parse("begin $str end")
end

clang_wrapper = parseall(read(open(joinpath(generated_dir, "libclang_api.jl")), String))
clang_wrapper = MacroTools.striplines(clang_wrapper)

function extract_api(clang_wrapper)
  apis = OrderedDict()
  for expr in clang_wrapper.args
    if expr.head != :function
      println("Skipping: ")
      println(expr)
      continue
    end

    julia_function = expr.args[1].args[1]
    julia_call_args = expr.args[1].args[2:end]

    function_body = expr.args[2]

    @capture(expr.args[2], ccall((c_symbol_, lib_), rettype_, c_signature_, c_call_args__) )

    @assert c_call_args == julia_call_args
    call_args = julia_call_args
    @assert julia_function == c_symbol.value
    function_name = julia_function

    c_signature = c_signature.args
    apis[function_name] = @eponymtuple(rettype, call_args, c_signature)
  end
  return apis
end

apis = extract_api(clang_wrapper)

# should be lazy.
function enumerate_api_args(apis)
  r = []
  for name in keys(apis)
    for (arg_i, arg) in enumerate(apis[name].call_args)
      index = @eponymtuple(name, arg_i)
      push!(r, @eponymtuple(index, arg))
    end
  end
  return r
end

function find_first_subsequence(needle, haystack)
  N = length(needle)
  if N > length(haystack)
    return nothing
  end
  for start = eachindex(haystack)
    end_ = start + N - 1
    if end_  > lastindex(haystack)
      continue
    end
    if all(needle .== haystack[start:end_]) # allows comparing tuples and vectors, but not nested
      return start
    end
  end
  return nothing
end

replacements = DefaultOrderedDict(()->OrderedDict())

# clipboard(join(unique(map(x->x.arg, enumerate_api_args(apis))), "\n")) to get all argument names

# but not timeOut
out_arg_names = [
  :keyOut
  :userDataIdOut
  :linkIndexOut
  :visualShapeIndexOut
  :outQuat
  :outPos
  :outOrn
  :angVelOut
  :vecOut
  :axisOut
  :valueOut
  :bodyIndicesOut
]

is_out_param(idx) = apis[idx.name].call_args[idx.arg_i] in out_arg_names

color_no_transparency = [
  :specularColor
  :lightColor
  :objectColorRGB
  :colorRGB
]

color_transparency = [:rgbaColor]

color_transparency_groups = [
  (:red, :green, :blue, :alpha) => :color
]

quaternion = [
  :startQuat,
  :endQuat,
  :quat,
  :orn,
  :rootOrn,
  :ornA,
  :ornB,
  :jointChildFrameOrn,
  :childOrientation,
  :collisionShapeOrientationA,
  :collisionShapeOrientationB,
  :targetOrientation,
  :baseOrientation,
  :baseInertialFrameOrientation,
]

quaternion_groups = [
  (:startOrnX, :startOrnY, :startOrnZ, :startOrnW) => :startOrn
]

# positions, and also dimensions
vector = [
  :rayFromWorld,
  :rayToWorld,
  :posA,
  :posB,
  :collisionShapePositionA,
  :collisionShapePositionB,
  :cameraTargetPosition,
  :basePosition,
  :baseInertialFramePosition,
  :position,
  :vec,
  :positionXYZ,
  :fromXYZ,
  :toXYZ,
  :childPosition,
  :cameraPosition,
]

vector_groups = [
  (:startPosX, :startPosY, :startPosZ)  =>  :startPos,
  (:rayFromWorldX, :rayFromWorldY, :rayFromWorldZ)  =>  :rayFromWorld,
  (:rayToWorldX, :rayToWorldY, :rayToWorldZ) => :rayToWorld,
  (:gravx, :gravy, :gravz) => :grav
]



# replace colors
for (idx, arg) in enumerate_api_args(apis)
  if is_out_param(idx)
    #println("Skipping out parameter: ")
    #@show idx arg
    continue
  end

  if arg in vcat(color_no_transparency, color_transparency)
    @assert @capture(apis[idx.name].c_signature[idx.arg_i], Ptr{T_})

    julian_arg = :color

    converter = if arg in color_transparency
      :($arg = bullet_color_alpha($T, $julian_arg))
    else
      :($arg = bullet_color($T, $julian_arg))
    end

    replacements[idx.name][idx.arg_i] = @eponymtuple(julian_arg, converter)
  end
end

# replace quaternion
for (idx, arg) in enumerate_api_args(apis)
  if is_out_param(idx)
    #println("Skipping out parameter: ")
    #@show idx arg
    continue
  end

  if arg in quaternion
    @assert @capture(apis[idx.name].c_signature[idx.arg_i], Ptr{T_})

    julian_arg = arg
    converter = :($arg = bullet_quaternion($T, $julian_arg))

    replacements[idx.name][idx.arg_i] = @eponymtuple(julian_arg, converter)
  end
end

# replace vector
for (idx, arg) in enumerate_api_args(apis)
  if is_out_param(idx)
    #println("Skipping out parameter: ")
    #@show idx arg
    continue
  end

  if arg in vector
    @assert @capture(apis[idx.name].c_signature[idx.arg_i], Ptr{T_})

    julian_arg = arg
    converter = :($arg = bullet_vector3($T, $julian_arg))

    replacements[idx.name][idx.arg_i] = @eponymtuple(julian_arg, converter)
  end
end


recursive_merge!(d1, d2...) = merge!(recursive_merge!, d1, d2...)
recursive_merge(d1, d2...) = merge!(recursive_merge, d1, d2...)

function process_group(apis, group_pairs, expander)
  replacements_group = DefaultOrderedDict(()->OrderedDict())
  for name in keys(apis)
    api = apis[name]
    for pair in group_pairs
      group = first(pair)
      julian_arg = last(pair)
      group_start = find_first_subsequence(group, api.call_args)
      if group_start === nothing
        continue
      end
      arg_i = group_start:(group_start + length(group) - 1)
      group_lhs = Expr(:tuple, group...)
      converter = :($group_lhs = $(expander)(Float64, $julian_arg))
      replacements_group[name][arg_i] = @eponymtuple(julian_arg, converter)
    end
  end
  return replacements_group
end



recursive_merge!(replacements,
  process_group(apis, color_transparency_groups, :bullet_color_alpha)
)

recursive_merge!(replacements,
  process_group(apis, vector_groups, :bullet_vector3)
)

recursive_merge!(replacements,
  process_group(apis, quaternion_groups, :bullet_quaternion)
)

function emit_safe_wrapper(safe_name, unsafe_name, unsafe_args, replacement_arguments)
  safe_args = []
  body = []

  for (arg_i, arg) in enumerate(unsafe_args)
    replaced_any = false
    for key in keys(replacement_arguments)
      if arg_i == first(key) # relies on punning between numbers and interval. (numbers are iterable)
        replacement = replacement_arguments[key]
        push!(safe_args, replacement.julian_arg)
        push!(body, replacement.converter)
        replaced_any = true
      elseif arg_i in key
        # already handled
        replaced_any = true
      end
    end
    if !replaced_any
      push!(safe_args, arg)
    end
  end

  e = :(
    function $(safe_name)($(safe_args...))
      $(body...)
      $unsafe_name($(unsafe_args...))
    end
  )
  return e
end

delete!(replacements, :b3RaycastBatchAddRays)

function make_safe_wrappers(replacements)
  output_exprs = []

  for unsafe_name in keys(replacements)
    replacement_arguments = replacements[unsafe_name]
    unsafe_args = apis[unsafe_name].call_args
    safe_name = Symbol(String(unsafe_name)[3:end])
    unsafe_name = Symbol("Raw." * String(unsafe_name))
    e = emit_safe_wrapper(safe_name, unsafe_name, unsafe_args, replacement_arguments)
    push!(output_exprs, e)
  end
  return output_exprs
end

all_wrappers = make_safe_wrappers(replacements)

module_expr = :(
  module Safe
  import ..Raw    # access the output of the Clang wrapper tool (`bullet_parse_headers.jl`)
  using ..Julian  # access the Julia/Bullet datatype converters
  $(all_wrappers...)
  end
)

module_expr = MacroTools.striplines(module_expr)

open(joinpath(generated_dir, "safe_api.jl"), "w") do f
  println(f, "#AAutogeneratd by: ", relpath(@__FILE__, package_dir))
  println(f, module_expr)
end

