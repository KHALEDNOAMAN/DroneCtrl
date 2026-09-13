import React, { useMemo } from 'react';
import { BackSide, Color, ShaderMaterial, Vector3 } from 'three';

const VERTEX = /* glsl */ `
varying vec3 vDirection;
void main() {
  vDirection = position;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}
`;

const FRAGMENT = /* glsl */ `
uniform vec3 topColor;
uniform vec3 midColor;
uniform vec3 horizonColor;
uniform vec3 groundColor;
uniform vec3 sunDirection;
uniform vec3 sunColor;
varying vec3 vDirection;

void main() {
  vec3 dir = normalize(vDirection);

  // Three stops: a narrow warm band hugging the horizon, a teal transition,
  // then deep blue overhead. Keeping the warm band tight is what stops the
  // whole upper half of the frame going orange.
  vec3 color = mix(horizonColor, midColor, smoothstep(0.0, 0.13, dir.y));
  color = mix(color, topColor, smoothstep(0.1, 0.5, dir.y));
  color = mix(groundColor, color, smoothstep(-0.16, 0.01, dir.y));

  // Sun disc plus the broad glow around it.
  float alignment = max(dot(dir, normalize(sunDirection)), 0.0);
  color += sunColor * pow(alignment, 220.0) * 1.4;
  color += sunColor * pow(alignment, 7.0) * 0.28;

  gl_FragColor = vec4(color, 1.0);
  #include <colorspace_fragment>
}
`;

interface Props {
  sunPosition: [number, number, number];
}

/**
 * Hand-rolled dusk gradient rather than drei's atmospheric Sky.
 *
 * The physical sky model needs careful turbidity and rayleigh tuning to avoid
 * washing out to near-white, and at that point the emissive gates and the
 * drone's LEDs stop reading against the background. A direct gradient gives a
 * dark, high-contrast sky that the neon elements sit on cleanly.
 */
export const GradientSky: React.FC<Props> = ({ sunPosition }) => {
  const material = useMemo(
    () =>
      new ShaderMaterial({
        vertexShader: VERTEX,
        fragmentShader: FRAGMENT,
        side: BackSide,
        depthWrite: false,
        fog: false,
        uniforms: {
          topColor: { value: new Color('#050f26') },
          midColor: { value: new Color('#1f4a63') },
          horizonColor: { value: new Color('#d9713f') },
          groundColor: { value: new Color('#08161c') },
          sunColor: { value: new Color('#ffcf8f') },
          sunDirection: { value: new Vector3(...sunPosition).normalize() },
        },
      }),
    [sunPosition],
  );

  return (
    <mesh material={material} scale={[-1, 1, 1]} renderOrder={-1000} frustumCulled={false}>
      <sphereGeometry args={[700, 32, 20]} />
    </mesh>
  );
};
