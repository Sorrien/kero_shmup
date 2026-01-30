use std::{
    collections::HashMap,
    fs::File,
    path::{Path, PathBuf},
    sync::Arc,
};

use cpal::traits::{DeviceTrait as _, HostTrait, StreamTrait as _};
use glamx::{Affine2, Vec2, Vec3};
use hecs::{Entity, World};
use oddio::{Frames, Spatial, SpatialScene};
use symphonia::core::{
    audio::SampleBuffer,
    codecs::DecoderOptions,
    formats::FormatOptions,
    io::{MediaSource, MediaSourceStream},
    meta::MetadataOptions,
    probe::Hint,
};

use crate::{
    AudioData, Camera,
    hierarchy::{ParentComponent, TransformComponent},
};

pub struct AudioSample {
    pub path: String,
    pub length_seconds: f32,
    pub index: usize,
}

pub fn audio_decode<M: MediaSource + 'static>(src: M, ext: Option<&str>) -> (Vec<f32>, usize, u32) {
    // Create the media source stream using the boxed media source from above.
    let mss = MediaSourceStream::new(Box::new(src), Default::default());

    // Create a hint to help the format registry guess what format reader is appropriate. In this
    // example we'll leave it empty.
    let mut hint = Hint::new();
    if let Some(ext) = ext {
        hint.with_extension(ext);
    }

    // Use the default options when reading and decoding.
    let format_opts: FormatOptions = Default::default();
    let metadata_opts: MetadataOptions = Default::default();
    let decoder_opts: DecoderOptions = Default::default();

    // Probe the media source stream for a format.
    let probed = symphonia::default::get_probe()
        .format(&hint, mss, &format_opts, &metadata_opts)
        .unwrap();

    // Get the format reader yielded by the probe operation.
    let mut format = probed.format;

    // Get the default track.
    let track = format.default_track().unwrap();

    // Create a decoder for the track.
    let mut decoder = symphonia::default::get_codecs()
        .make(&track.codec_params, &decoder_opts)
        .unwrap();

    // Store the track identifier, we'll use it to filter packets.
    let track_id = track.id;

    let mut sample_count = 0;
    let mut sample_buf = None;
    let mut rate = 0;

    let mut samples = Vec::new();

    loop {
        // Get the next packet from the media format.
        let packet = match format.next_packet() {
            Ok(packet) => packet,
            Err(symphonia::core::errors::Error::ResetRequired) => {
                // The track list has been changed. Re-examine it and create a new set of decoders,
                // then restart the decode loop. This is an advanced feature and it is not
                // unreasonable to consider this "the end." As of v0.5.0, the only usage of this is
                // for chained OGG physical streams.
                unimplemented!();
            }
            Err(symphonia::core::errors::Error::IoError(err)) => {
                match err.kind() {
                    std::io::ErrorKind::UnexpectedEof => {
                        break;
                    }
                    _ => {
                        // A unrecoverable error occured, halt decoding.
                        panic!("{}", err);
                    }
                }
            }
            Err(err) => {
                // A unrecoverable error occured, halt decoding.
                panic!("{}", err);
            }
        };

        // If the packet does not belong to the selected track, skip it.
        if packet.track_id() != track_id {
            continue;
        }

        // Decode the packet into audio samples, ignoring any decode errors.
        match decoder.decode(&packet) {
            Ok(audio_buf) => {
                // The decoded audio samples may now be accessed via the audio buffer if per-channel
                // slices of samples in their native decoded format is desired. Use-cases where
                // the samples need to be accessed in an interleaved order or converted into
                // another sample format, or a byte buffer is required, are covered by copying the
                // audio buffer into a sample buffer or raw sample buffer, respectively. In the
                // example below, we will copy the audio buffer into a sample buffer in an
                // interleaved order while also converting to a f32 sample format.

                // If this is the *first* decoded packet, create a sample buffer matching the
                // decoded audio buffer format.
                if sample_buf.is_none() {
                    // Get the audio buffer specification.
                    let spec = *audio_buf.spec();
                    rate = spec.rate;

                    // Get the capacity of the decoded buffer. Note: This is capacity, not length!
                    let duration = audio_buf.capacity() as u64;

                    // Create the f32 sample buffer.
                    sample_buf = Some(SampleBuffer::<f32>::new(duration, spec));
                }

                // Copy the decoded audio buffer into the sample buffer in an interleaved format.
                if let Some(buf) = &mut sample_buf {
                    buf.copy_interleaved_ref(audio_buf);

                    // The samples may now be access via the `samples()` function.
                    sample_count += buf.samples().len();
                    samples.append(&mut buf.samples().to_vec());
                }
            }
            Err(symphonia::core::errors::Error::DecodeError(_)) => (),
            Err(_) => break,
        }
    }

    (samples, sample_count, rate)
}

pub fn load_audio<P>(
    paths: Vec<P>,
) -> Result<
    (
        Vec<Arc<Frames<f32>>>,
        Vec<AudioSample>,
        HashMap<String, usize>,
    ),
    std::io::Error,
>
where
    P: AsRef<Path>,
{
    let mut sounds = Vec::new();
    let mut sound_metadata = Vec::new();
    for (i, path) in paths.iter().enumerate() {
        let file = File::open(&path)?;

        let mut path_buf = PathBuf::new();
        path_buf.push(path);
        let ext = path_buf.extension().unwrap().to_str();
        let (samples, length_samples, source_sample_rate) = audio_decode(file, ext);
        let length_samples = length_samples as f32 / 2.;
        let length_seconds = length_samples as f32 / source_sample_rate as f32;

        let sound_frames = oddio::Frames::from_slice(source_sample_rate, &samples);
        sounds.push(sound_frames);

        sound_metadata.push(AudioSample {
            path: path_buf.as_os_str().to_os_string().into_string().unwrap(),
            length_seconds,
            index: i,
        });
    }

    let map = HashMap::from_iter(
        sound_metadata
            .iter()
            .map(|sound| (sound.path.clone(), sound.index)),
    );

    Ok((sounds, sound_metadata, map))
}

pub struct AudioEmitter {
    spatial: oddio::Spatial,
}

pub fn update_audio_emitters(camera: &Camera, world: &mut World, to_despawn: &mut Vec<Entity>) {
    let mut pending = world
        .query::<(Entity, &AudioEmitter)>()
        .iter()
        .filter_map(|(entity, emitter)| {
            if emitter.spatial.is_finished() {
                Some(entity)
            } else {
                None
            }
        })
        .collect::<Vec<_>>();
    to_despawn.append(&mut pending);

    for (transform, emitter) in world.query_mut::<(&TransformComponent, &mut AudioEmitter)>() {
        //todo: at some point velocity and continuity should be handled. Knowing the previous position/transform will be necessary for this.
        emitter.spatial.set_motion(
            (translate_sound_2d(
                Vec2::new(camera.position.x, camera.position.y),
                transform.transform.translation,
            ))
            .extend(0.0)
            .into(),
            Vec3::ZERO.into(),
            true,
        );
    }
}

pub fn translate_sound_2d(listener_pos: Vec2, sound_pos: Vec2) -> Vec2 {
    sound_pos - listener_pos
}

pub fn translate_sound_3d(listener_pos: Vec3, sound_pos: Vec3) -> Vec3 {
    sound_pos - listener_pos
}

pub fn setup_output_stream(
    mut signal: SpatialScene,
) -> (cpal::Stream, u32, cpal::Host, cpal::Device) {
    let host = cpal::default_host();

    let device = host
        .default_output_device()
        .expect("no output device available!");
    let sample_rate = device.default_output_config().unwrap().sample_rate();
    let stream_config = cpal::StreamConfig {
        channels: device.default_output_config().unwrap().channels(),
        sample_rate,
        buffer_size: cpal::BufferSize::Default,
    };

    // We send `scene` into this closure, where changes to `scene_handle` are reflected.
    // `scene_handle` is how we add new sounds and modify the scene live.
    let stream = device
        .build_output_stream(
            &stream_config,
            move |data: &mut [f32], _: &cpal::OutputCallbackInfo| {
                let frames = oddio::frame_stereo(data);
                oddio::run(&mut signal, sample_rate, frames);
            },
            move |err| {
                eprintln!("{}", err);
            },
            None,
        )
        .unwrap();
    stream.play().unwrap();

    (stream, sample_rate, host, device)
}

pub fn spawn_sound(
    audio_emitters_to_spawn: &mut Vec<(TransformComponent, Spatial)>,
    parented_audio_emitters_to_spawn: &mut Vec<(TransformComponent, Spatial, ParentComponent)>,
    cam_pos: Vec2,
    position: Vec2,
    radius: f32,
    max_distance: f32,
    audio: &mut AudioData,
    sound_frames: Arc<oddio::Frames<f32>>,
    parent: Option<(Entity, Vec2)>,
) {
    let signal = oddio::FramesSignal::from(sound_frames.clone());

    let spatial_position = translate_sound_2d(cam_pos, position);
    let spatial_control = audio.sound_scene_handle.play_buffered(
        signal,
        oddio::SpatialOptions {
            position: spatial_position.extend(0.).into(),
            velocity: Vec3::ZERO.into(),
            radius,
        },
        max_distance,
        audio.sample_rate,
        0.1,
    );

    if let Some((parent, relative)) = parent {
        parented_audio_emitters_to_spawn.push((
            TransformComponent {
                transform: Affine2::from_translation(position),
            },
            spatial_control,
            ParentComponent {
                parent,
                local_transform: Affine2::from_translation(relative),
            },
        ));
    } else {
        audio_emitters_to_spawn.push((
            TransformComponent {
                transform: Affine2::from_translation(position),
            },
            spatial_control,
        ));
    }
}
